#include "scene.h"
#include "matrices.h"

std::vector<Instance> g_Instances;

glm::vec3 g_LampP0(0.0f);
glm::vec3 g_LampP1(0.0f);
glm::vec3 g_Lamp2Pos(0.0f);
int       g_BezierInstance = -1;
int       g_ButtonInstance = -1;
int       g_DoorInstance   = -1;

Material MakeMaterial(int texture_id, int uv_mode, float uv_scale,
                      float ks, float shininess,
                      int pattern, float emissive,
                      int shading)
{
    Material m;
    m.texture_id = texture_id;
    m.uv_mode    = uv_mode;
    m.uv_scale   = uv_scale;
    m.ks         = ks;
    m.shininess  = shininess;
    m.pattern    = pattern;
    m.emissive   = emissive;
    m.shading    = shading;
    return m;
}

void SceneWorldBounds(glm::vec3* out_min, glm::vec3* out_max)
{
    const float HX = Room::HALF_X;
    const float HZ = Room::HALF_Z;
    const float H  = Room::HEIGHT;
    const float T  = Room::THICKNESS;

    // Margem generosa em +X porque o patamar externo da saida fica para fora
    // da parede; o resto segue a caixa da sala.
    if (out_min) *out_min = glm::vec3(-HX - T - 0.5f, -T - 0.5f, -HZ - T - 0.5f);
    if (out_max) *out_max = glm::vec3( HX + T + 2.5f,  H  + T + 0.5f,  HZ + T + 0.5f);
}

glm::mat4 InstanceModelMatrix(const Instance& inst)
{
    // T * Ry * S. A ordem importa: a escala e aplicada primeiro (mais a
    // direita), depois a rotacao, e so entao a translacao.
    return Matrix_Translate(inst.pos.x, inst.pos.y, inst.pos.z)
         * Matrix_Rotate_Y(inst.yaw)
         * Matrix_Scale(inst.scale.x, inst.scale.y, inst.scale.z);
}

// Auxiliar para nao repetir a construcao de cada caixa.
static Instance Box(const glm::vec3& center, const glm::vec3& size,
                    const Material& mat, bool solid)
{
    Instance i;
    i.mesh         = "the_cube";
    i.pos          = center;
    i.yaw          = 0.0f;
    i.scale        = size;     // o cubo tem lado 1, entao escala == tamanho em metros
    i.mat          = mat;
    i.solid        = solid;
    i.grabbable    = false;
    i.casts_shadow = false;    // casca da sala e luminarias: ver scene.h
    return i;
}

void BuildScene(std::vector<Instance>& out)
{
    out.clear();
    g_BezierInstance = -1;
    g_ButtonInstance = -1;
    g_DoorInstance   = -1;

    const float HX = Room::HALF_X;
    const float HZ = Room::HALF_Z;
    const float H  = Room::HEIGHT;
    const float T  = Room::THICKNESS;

    // Largura externa, usada em piso e teto para fechar os cantos.
    const float OUT_X = 2.0f*HX + 2.0f*T;
    const float OUT_Z = 2.0f*HZ + 2.0f*T;

    // -----------------------------------------------------------------------
    // Materiais do cenario.
    //
    // Parede, piso e teto nao usam imagem: a cor vem de uma textura
    // PROCEDURAL avaliada no fragment shader, como o SPEC.md especifica. A
    // coordenada continua sendo UV_WORLD (em metros), entao os paineis e a
    // grade tem o mesmo tamanho em qualquer parede, independente da escala da
    // caixa que a desenha.
    // -----------------------------------------------------------------------
    Material wall_mat    = MakeMaterial(0, UV_WORLD, 1.0f, 0.04f, 20.0f, PATTERN_WALL);
    Material floor_mat   = MakeMaterial(0, UV_WORLD, 1.0f, 0.06f, 28.0f, PATTERN_FLOOR);
    Material ceiling_mat = MakeMaterial(0, UV_WORLD, 1.0f, 0.02f, 12.0f, PATTERN_CEILING);

    // Luminaria: material emissivo. Nao recebe iluminacao nem sombra, e e o
    // unico objeto que o jogador le como "a fonte de luz".
    Material lamp_mat    = MakeMaterial(0, UV_OBJ, 1.0f, 0.0f, 1.0f, PATTERN_TEXTURE, 1.9f);

    // --- piso e teto ---
    out.push_back(Box(glm::vec3(0.0f, -T*0.5f,     0.0f), glm::vec3(OUT_X, T, OUT_Z), floor_mat,   true));
    out.push_back(Box(glm::vec3(0.0f,  H + T*0.5f, 0.0f), glm::vec3(OUT_X, T, OUT_Z), ceiling_mat, true));

    // --- paredes inteiras ---
    out.push_back(Box(glm::vec3(0.0f, H*0.5f, -HZ - T*0.5f), glm::vec3(OUT_X, H, T), wall_mat, true)); // fundo (-Z)
    out.push_back(Box(glm::vec3(0.0f, H*0.5f,  HZ + T*0.5f), glm::vec3(OUT_X, H, T), wall_mat, true)); // frente (+Z)
    out.push_back(Box(glm::vec3(-HX - T*0.5f, H*0.5f, 0.0f), glm::vec3(T, H, 2.0f*HZ), wall_mat, true)); // esquerda (-X)

    // --- parede da direita (+X), dividida em tres para abrir a saida ---
    {
        const float hw      = Room::EXIT_WIDTH * 0.5f;
        const float side_w  = HZ - hw;
        const float side_cz = hw + side_w*0.5f;

        out.push_back(Box(glm::vec3(HX + T*0.5f, Room::EXIT_Y0*0.5f, 0.0f),
                          glm::vec3(T, Room::EXIT_Y0, Room::EXIT_WIDTH), wall_mat, true));
        out.push_back(Box(glm::vec3(HX + T + 0.6f, Room::EXIT_Y0 - T*0.5f, 0.0f),
                          glm::vec3(1.2f, T, Room::EXIT_WIDTH + 0.4f), floor_mat, true));
        // -------------------------------------------------------------------
        // Porta da saida (M6). Preenche o vao inteiro, do peitoril (EXIT_Y0)
        // ao teto, no meio da espessura da parede. Ao abrir, desce para
        // dentro do pedaco de parede abaixo do vao (ver level.cpp).
        // -------------------------------------------------------------------
        {
            const float door_h = H - Room::EXIT_Y0;
            Instance door = Box(glm::vec3(HX + T*0.5f, Room::EXIT_Y0 + door_h*0.5f, 0.0f),
                                glm::vec3(Puzzle::DOOR_THICKNESS, door_h, Room::EXIT_WIDTH),
                                MakeMaterial(0, UV_OBJ, 1.0f, 0.30f, 40.0f, PATTERN_DOOR),
                                true);
            g_DoorInstance = (int)out.size();
            out.push_back(door);
        }

        // -------------------------------------------------------------------
        // Corredor do lado de fora, sobre a plataforma: duas paredes laterais,
        // teto e, no fundo, um painel luminoso que marca a saida. Chegar nele
        // conclui a fase.
        // -------------------------------------------------------------------
        {
            const float L     = Puzzle::CORRIDOR_LENGTH;
            const float y0    = Room::EXIT_Y0;
            const float ch    = H - y0;               // altura interna
            const float x_mid = HX + T + L*0.5f;

            out.push_back(Box(glm::vec3(x_mid, y0 + ch*0.5f, -(hw + T*0.5f)),
                              glm::vec3(L, ch, T), wall_mat, true));
            out.push_back(Box(glm::vec3(x_mid, y0 + ch*0.5f,  (hw + T*0.5f)),
                              glm::vec3(L, ch, T), wall_mat, true));
            out.push_back(Box(glm::vec3(x_mid + T*0.5f, H + T*0.5f, 0.0f),
                              glm::vec3(L + T, T, Room::EXIT_WIDTH + 2.0f*T), ceiling_mat, true));

            Material exit_mat = MakeMaterial(0, UV_OBJ, 1.0f, 0.0f, 1.0f,
                                             PATTERN_TEXTURE, 1.6f);
            out.push_back(Box(glm::vec3(HX + T + L + 0.05f, y0 + ch*0.5f, 0.0f),
                              glm::vec3(0.10f, ch, Room::EXIT_WIDTH), exit_mat, true));
        }

        out.push_back(Box(glm::vec3(HX + T*0.5f, H*0.5f, -side_cz),
                          glm::vec3(T, H, side_w), wall_mat, true));
        out.push_back(Box(glm::vec3(HX + T*0.5f, H*0.5f,  side_cz),
                          glm::vec3(T, H, side_w), wall_mat, true));
    }

    // -----------------------------------------------------------------------
    // Luminarias embutidas no teto.
    //
    // Ficam 1 cm abaixo da face inferior do teto: encostadas exatamente nela,
    // as duas superficies seriam coplanares e brigariam no z-buffer
    // (z-fighting), que e bug listado nominalmente nos criterios.
    //
    // Nao sao solidas e nao entram no passo de profundidade: uma fonte de luz
    // nao projeta sombra de si mesma, e como estao no teto elas sombreariam a
    // sala inteira.
    // -----------------------------------------------------------------------
    {
        const float lamp_y    = H - 0.06f;
        const float strip_len = 2.0f*HX * 0.62f * 0.10f; // 10% do comprimento original
        const float strip_wid = 1.10f * 0.10f;

        out.push_back(Box(glm::vec3(0.0f, lamp_y, 0.0f),
                          glm::vec3(strip_len, 0.10f, strip_wid),
                          lamp_mat, false));

        g_LampP0 = glm::vec3(-strip_len*0.5f, lamp_y - 0.05f, 0.0f);
        g_LampP1 = glm::vec3( strip_len*0.5f, lamp_y - 0.05f, 0.0f);
    }

    // -----------------------------------------------------------------------
    // Segunda luminaria: pendente em cone na quina oposta a porta.
    // -----------------------------------------------------------------------
    {
        const float cone_d = 2.0f*HX * 0.62f * 0.10f;
        const float cone_h = cone_d * 0.60f;

        glm::vec3 cone_pos(-HX + 1.1f, H - 0.55f, -HZ + 1.1f);

        Instance cone;
        cone.mesh         = "the_cone";
        cone.pos          = cone_pos;
        cone.yaw          = 0.0f;
        cone.scale        = glm::vec3(cone_d, cone_h, cone_d);
        cone.mat          = lamp_mat;
        cone.solid        = false;
        cone.grabbable    = false;
        cone.casts_shadow = false;
        out.push_back(cone);

        // A luz sai da boca do cone, na base dele.
        g_Lamp2Pos = cone_pos - glm::vec3(0.0f, cone_h*0.5f + 0.03f, 0.0f);
    }

    // -----------------------------------------------------------------------
    // Objetos da fase (M5 - Assets e luz):
    //   - TextureImage0: red_brick_diff_1k.jpg
    //   - TextureImage1: rocky_terrain_02_diff_1k.jpg
    //   - TextureImage2: die_diff.jpg (atlas 6 faces do dado)
    //   - TextureImage3: rubik_diff.jpg (atlas 6 faces do cubo magico)
    //   - TextureImage4: pawn_wood_diff.jpg (madeira torneada do peao)
    // -----------------------------------------------------------------------

    // 1) Tres instancias da MESMA malha "the_die" (dado chanfrado com as 6
    //    faces mapeadas por UV_OBJ), cada uma com sua propria matriz Model:
    //    atende o requisito de instancias de objetos com malha/textura de dado.
    {
        Material die_mat = MakeMaterial(2, UV_OBJ, 1.0f, 0.30f, 44.0f,
                                        PATTERN_TEXTURE, 0.0f, SHADING_PHONG);
        const float die_s[3]   = { 0.34f, 0.30f, 0.38f };
        const float die_yaw[3] = { 0.25f, 0.95f, -0.50f };
        const glm::vec3 die_pos[3] = {
            glm::vec3( 0.2f, die_s[0]*0.5f,  0.4f),
            glm::vec3( 2.4f, die_s[1]*0.5f,  1.9f),
            glm::vec3(-2.6f, die_s[2]*0.5f, -1.7f)
        };
        for (int k = 0; k < 3; ++k)
        {
            Instance die;
            die.mesh         = "the_die";
            die.pos          = die_pos[k];
            die.yaw          = die_yaw[k];
            die.scale        = glm::vec3(die_s[k]);
            die.mat          = die_mat;
            die.solid        = true;
            die.grabbable    = true;
            die.casts_shadow = true;
            out.push_back(die);
        }
    }

    // 2) Duas instancias da malha "the_rubik" (cubo magico 3x3x3 modelado com
    //    sulcos e cantos chanfrados, mapeado com rubik_diff.jpg).
    {
        Material rubik_mat = MakeMaterial(3, UV_OBJ, 1.0f, 0.38f, 56.0f,
                                          PATTERN_TEXTURE, 0.0f, SHADING_PHONG);
        const float rubik_s[2]   = { 0.36f, 0.32f };
        const float rubik_yaw[2] = { -0.40f, 0.75f };
        const glm::vec3 rubik_pos[2] = {
            glm::vec3( 1.5f, rubik_s[0]*0.5f, -1.4f),
            glm::vec3(-1.2f, rubik_s[1]*0.5f,  2.1f)
        };
        for (int k = 0; k < 2; ++k)
        {
            Instance rubik;
            rubik.mesh         = "the_rubik";
            rubik.pos          = rubik_pos[k];
            rubik.yaw          = rubik_yaw[k];
            rubik.scale        = glm::vec3(rubik_s[k]);
            rubik.mat          = rubik_mat;
            rubik.solid        = true;
            rubik.grabbable    = true;
            rubik.casts_shadow = true;
            out.push_back(rubik);
        }
    }

    // 3) Duas instancias do peao de xadrez ("the_pawn", gerado por revolucao
    //    com 48 segmentos e 5664 triangulos):
    //    - A primeira usa Blinn-Phong por fragmento (SHADING_PHONG);
    //    - A segunda, posicionada ao lado, usa interpolacao de Gouraud por
    //      vertice (SHADING_GOURAUD), permitindo comparar diretamente os dois
    //      modelos de interpolacao sobre a mesma geometria curva e brilhante.
    {
        const float pawn_h = 0.56f;
        Instance pawn_phong;
        pawn_phong.mesh         = "the_pawn";
        pawn_phong.pos          = glm::vec3(-0.6f, pawn_h*0.5f, -1.2f);
        pawn_phong.yaw          = 0.0f;
        pawn_phong.scale        = glm::vec3(pawn_h);
        pawn_phong.mat          = MakeMaterial(4, UV_OBJ, 1.0f, 0.35f, 48.0f,
                                               PATTERN_TEXTURE, 0.0f, SHADING_PHONG);
        pawn_phong.solid        = true;
        pawn_phong.grabbable    = true;
        pawn_phong.casts_shadow = true;
        out.push_back(pawn_phong);

        Instance pawn_gouraud = pawn_phong;
        pawn_gouraud.pos          = glm::vec3(-1.4f, pawn_h*0.5f, -1.2f);
        pawn_gouraud.mat          = MakeMaterial(4, UV_OBJ, 1.0f, 0.35f, 48.0f,
                                                 PATTERN_TEXTURE, 0.0f, SHADING_GOURAUD);
        out.push_back(pawn_gouraud);
    }

    // 4) Bloco de alvenaria ("the_cube" com TextureImage0) para manter as 5
    //    imagens de textura ativas em objetos pegaveis da cena.
    {
        const float cs = 0.36f;
        Instance brick_box = Box(glm::vec3(2.8f, cs*0.5f, -0.2f),
                                 glm::vec3(cs),
                                 MakeMaterial(0, UV_OBJ, 1.0f, 0.12f, 20.0f,
                                              PATTERN_TEXTURE, 0.0f, SHADING_PHONG),
                                 true);
        brick_box.yaw          = 0.18f;
        brick_box.grabbable    = true;
        brick_box.casts_shadow = true;
        out.push_back(brick_box);
    }

    // 5) Coelho / boneco CC0 pousado sobre o piso (a malha tem bbox_min.y ~ -0.99,
    //    entao pos.y == scale.y apoia a base em y = 0) e pegavel na mecanica.
    {
        Instance bunny;
        bunny.mesh         = "the_bunny";
        bunny.pos          = glm::vec3(-3.8f, 0.32f, 2.4f);
        bunny.yaw          = -0.6f;
        bunny.scale        = glm::vec3(0.32f);
        bunny.mat          = MakeMaterial(1, UV_PLANAR_XY, 1.0f, 0.18f, 24.0f,
                                          PATTERN_TEXTURE, 0.0f, SHADING_PHONG);
        bunny.solid        = true;
        bunny.grabbable    = true;
        bunny.casts_shadow = true;
        out.push_back(bunny);
    }

    // 6) Objeto decorativo flutuante cuja movimentacao e definida por uma curva
    //    de Bezier cubica (SPEC.md e M3): nao e solido nem pode ser pego, e
    //    tambem utiliza interpolacao de Gouraud (SHADING_GOURAUD).
    {
        Instance sphere;
        sphere.mesh         = "the_sphere";
        sphere.pos          = glm::vec3(-4.2f, 2.6f, -3.2f);
        sphere.yaw          = 0.0f;
        sphere.scale        = glm::vec3(0.25f);
        sphere.mat          = MakeMaterial(1, UV_SPHERE, 1.0f, 0.40f, 48.0f,
                                           PATTERN_TEXTURE, 0.0f, SHADING_GOURAUD);
        sphere.solid        = false;
        sphere.grabbable    = false;
        sphere.casts_shadow = true;
        g_BezierInstance = (int)out.size();
        out.push_back(sphere);
    }

    // 7) Botao do puzzle (M6): placa fina no piso, a frente da saida. Nao e
    //    solida, para nao virar um degrau que trave a caminhada; quem decide
    //    se ha algo em cima e level.cpp, por teste de sobreposicao de AABBs.
    {
        Instance button = Box(glm::vec3(Puzzle::BUTTON_X, Puzzle::BUTTON_UP_H*0.5f, Puzzle::BUTTON_Z),
                              glm::vec3(Puzzle::BUTTON_SIZE, Puzzle::BUTTON_UP_H, Puzzle::BUTTON_SIZE),
                              MakeMaterial(0, UV_OBJ, 1.0f, 0.25f, 36.0f, PATTERN_BUTTON_IDLE),
                              false);
        g_ButtonInstance = (int)out.size();
        out.push_back(button);
    }
}
