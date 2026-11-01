#include "scene.h"
#include "matrices.h"

std::vector<Instance> g_Instances;

glm::vec3 g_LampP0(0.0f);
glm::vec3 g_LampP1(0.0f);
glm::vec3 g_Lamp2Pos(0.0f);
int                   g_BezierInstance = -1;

Material MakeMaterial(int texture_id, int uv_mode, float uv_scale,
                      float ks, float shininess,
                      int pattern, float emissive)
{
    Material m;
    m.texture_id = texture_id;
    m.uv_mode    = uv_mode;
    m.uv_scale   = uv_scale;
    m.ks         = ks;
    m.shininess  = shininess;
    m.pattern    = pattern;
    m.emissive   = emissive;
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
    i.mesh      = "the_cube";
    i.pos       = center;
    i.yaw       = 0.0f;
    i.scale     = size;     // o cubo tem lado 1, entao escala == tamanho em metros
    i.mat       = mat;
    i.solid        = solid;
    i.grabbable    = false;
    i.casts_shadow = false;   // casca da sala e luminarias: ver scene.h
    return i;
}

void BuildScene(std::vector<Instance>& out)
{
    out.clear();
    g_BezierInstance = -1;

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
        // UMA luminaria so, correndo ao longo do eixo maior da sala.
        //
        // Duas faixas separadas entravam em contradicao com o modelo de
        // iluminacao: o SPEC.md define "uma luz principal", e com uma unica
        // direcao de luz todas as sombras apontam para o mesmo lado, o que
        // nao corresponde ao que o jogador ve se houver dois paineis. Uma
        // faixa unica e coerente, e e tambem o que aparece na imagem de
        // referencia.
        //
        // A luz esta inclinada principalmente no eixo Z, o curto: e assim que
        // uma luminaria em faixa se comporta de verdade, espalhando a sombra
        // perpendicularmente ao seu comprimento.
        const float lamp_y    = H - 0.06f;
        const float strip_len = 2.0f*HX * 0.62f * 0.10f; // 10% do comprimento original
        const float strip_wid = 1.10f * 0.10f;

        out.push_back(Box(glm::vec3(0.0f, lamp_y, 0.0f),
                          glm::vec3(strip_len, 0.10f, strip_wid),
                          lamp_mat, false));

        // Segmento usado pela iluminacao. Fica logo abaixo da face inferior
        // da faixa, que e de onde a luz sai de fato. Com a faixa curta ele
        // quase vira um ponto, mas a formula do ponto mais proximo continua
        // valendo sem nenhuma mudanca.
        g_LampP0 = glm::vec3(-strip_len*0.5f, lamp_y - 0.05f, 0.0f);
        g_LampP1 = glm::vec3( strip_len*0.5f, lamp_y - 0.05f, 0.0f);
    }

    // -----------------------------------------------------------------------
    // Segunda luminaria: pendente em cone na quina oposta a porta.
    //
    // A saida fica na parede +X. Olhando para ela, a esquerda e o sentido -Z,
    // entao a quina pedida e a de -X com -Z. A malha "the_cone" e um solido de
    // revolucao gerado por nos, com o apice para cima, no formato de uma
    // cupula de lampada pendente.
    // -----------------------------------------------------------------------
    {
        const float cone_d = 2.0f*HX * 0.62f * 0.10f; // mesmo tamanho da faixa nova
        const float cone_h = cone_d * 0.60f;

        glm::vec3 cone_pos(-HX + 1.1f, H - 0.55f, -HZ + 1.1f);

        Instance cone;
        cone.mesh         = "the_cone";
        cone.pos          = cone_pos;
        cone.yaw          = 0.0f;
        cone.scale        = glm::vec3(cone_d, cone_h, cone_d);
        cone.mat          = lamp_mat;   // mesma intensidade da faixa central
        cone.solid        = false;
        cone.grabbable    = false;
        cone.casts_shadow = false;
        out.push_back(cone);

        // A luz sai da boca do cone, na base dele.
        g_Lamp2Pos = cone_pos - glm::vec3(0.0f, cone_h*0.5f + 0.03f, 0.0f);
    }

    // -----------------------------------------------------------------------
    // Objetos da fase.
    // -----------------------------------------------------------------------

    // Tres caixas da MESMA malha "the_cube", cada uma com a sua Model matrix:
    // e isto que atende o requisito de instancias de objetos.
    Material crate_mat = MakeMaterial(0, UV_OBJ, 1.0f, 0.20f, 32.0f);
    const float cs = 0.32f; // aresta da caixa, em metros
    const glm::vec3 crate_size(cs, cs, cs);
    const glm::vec3 crate_pos[3] = {
        glm::vec3( 0.2f, cs*0.5f,  0.4f),
        glm::vec3( 2.4f, cs*0.5f,  1.9f),
        glm::vec3(-2.8f, cs*0.5f, -1.7f)
    };
    for (int k = 0; k < 3; ++k)
    {
        Instance crate = Box(crate_pos[k], crate_size, crate_mat, true);
        crate.yaw          = 0.35f * k;
        crate.grabbable    = true;
        crate.casts_shadow = true;
        out.push_back(crate);
    }

    // Coelho pousado sobre o piso (a malha tem bbox_min.y ~ -0.99, entao
    // pos.y == scale.y apoia a base em y = 0) e pegavel na mecanica.
    {
        Instance bunny;
        bunny.mesh      = "the_bunny";
        bunny.pos       = glm::vec3(-3.8f, 0.32f, 2.4f);
        bunny.yaw       = -0.6f;
        bunny.scale     = glm::vec3(0.32f);
        bunny.mat       = MakeMaterial(1, UV_PLANAR_XY, 1.0f, 0.15f, 20.0f);
        bunny.solid        = true;
        bunny.grabbable    = true;
        bunny.casts_shadow = true;
        out.push_back(bunny);
    }

    // Objeto decorativo flutuante cuja movimentacao e definida por uma curva
    // de Bezier cubica (SPEC.md e M3): nao e solido nem pode ser pego.
    {
        Instance sphere;
        sphere.mesh      = "the_sphere";
        sphere.pos       = glm::vec3(-4.2f, 2.6f, -3.2f);
        sphere.yaw       = 0.0f;
        sphere.scale     = glm::vec3(0.25f);
        sphere.mat       = MakeMaterial(1, UV_SPHERE, 1.0f, 0.35f, 48.0f);
        sphere.solid        = false;
        sphere.grabbable    = false;
        sphere.casts_shadow = true;
        g_BezierInstance = (int)out.size();
        out.push_back(sphere);
    }
}
