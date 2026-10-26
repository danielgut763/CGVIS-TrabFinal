#include "scene.h"
#include "matrices.h"

std::vector<Instance> g_Instances;

Material MakeMaterial(int texture_id, int uv_mode, float uv_scale,
                      float ks, float shininess)
{
    Material m;
    m.texture_id = texture_id;
    m.uv_mode    = uv_mode;
    m.uv_scale   = uv_scale;
    m.ks         = ks;
    m.shininess  = shininess;
    return m;
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
    i.solid     = solid;
    i.grabbable = false;
    return i;
}

void BuildScene(std::vector<Instance>& out)
{
    out.clear();

    const float HX = Room::HALF_X;
    const float HZ = Room::HALF_Z;
    const float H  = Room::HEIGHT;
    const float T  = Room::THICKNESS;

    // Largura externa, usada em piso e teto para fechar os cantos.
    const float OUT_X = 2.0f*HX + 2.0f*T;
    const float OUT_Z = 2.0f*HZ + 2.0f*T;

    // Paredes e teto usam a textura 0 (tijolo); piso usa a 1 (terreno).
    // UV_WORLD com uv_scale em METROS garante texel quadrado em todas elas.
    Material wall_mat    = MakeMaterial(0, UV_WORLD, 2.0f, 0.05f, 16.0f);
    Material floor_mat   = MakeMaterial(1, UV_WORLD, 2.0f, 0.03f, 12.0f);
    Material ceiling_mat = MakeMaterial(1, UV_WORLD, 2.0f, 0.03f, 12.0f);

    // --- piso e teto ---
    out.push_back(Box(glm::vec3(0.0f, -T*0.5f,   0.0f), glm::vec3(OUT_X, T, OUT_Z), floor_mat,   true));
    out.push_back(Box(glm::vec3(0.0f,  H + T*0.5f, 0.0f), glm::vec3(OUT_X, T, OUT_Z), ceiling_mat, true));

    // --- paredes inteiras ---
    out.push_back(Box(glm::vec3(0.0f, H*0.5f, -HZ - T*0.5f), glm::vec3(OUT_X, H, T), wall_mat, true)); // fundo (-Z)
    out.push_back(Box(glm::vec3(0.0f, H*0.5f,  HZ + T*0.5f), glm::vec3(OUT_X, H, T), wall_mat, true)); // frente (+Z)
    out.push_back(Box(glm::vec3(-HX - T*0.5f, H*0.5f, 0.0f), glm::vec3(T, H, 2.0f*HZ), wall_mat, true)); // esquerda (-X)

    // --- parede da direita (+X), dividida em tres para abrir a saida ---
    // O vao fica entre z = -EXIT_WIDTH/2 e +EXIT_WIDTH/2, de y = EXIT_Y0 ate o teto.
    {
        const float hw      = Room::EXIT_WIDTH * 0.5f;   // meia largura do vao
        const float side_w  = HZ - hw;                   // largura de cada lateral
        const float side_cz = hw + side_w*0.5f;          // centro em Z das laterais

        // trecho abaixo do vao (e o degrau que o jogador precisa alcancar)
        out.push_back(Box(glm::vec3(HX + T*0.5f, Room::EXIT_Y0*0.5f, 0.0f),
                          glm::vec3(T, Room::EXIT_Y0, Room::EXIT_WIDTH), wall_mat, true));
        // laterais, de altura cheia
        out.push_back(Box(glm::vec3(HX + T*0.5f, H*0.5f, -side_cz),
                          glm::vec3(T, H, side_w), wall_mat, true));
        out.push_back(Box(glm::vec3(HX + T*0.5f, H*0.5f,  side_cz),
                          glm::vec3(T, H, side_w), wall_mat, true));
    }

    // --- objetos de teste ---

    // Tres caixas da MESMA malha "the_cube", cada uma com a sua Model matrix.
    // E isto que atende o requisito de instancias de objetos.
    Material crate_mat = MakeMaterial(0, UV_OBJ, 1.0f, 0.2f, 32.0f);
    const float cs = 0.5f; // aresta da caixa, em metros
    const glm::vec3 crate_size(cs, cs, cs);
    const glm::vec3 crate_pos[3] = {
        glm::vec3( 0.0f, cs*0.5f,  0.0f),
        glm::vec3( 1.4f, cs*0.5f,  1.2f),
        glm::vec3(-1.6f, cs*0.5f, -1.0f)
    };
    for (int k = 0; k < 3; ++k)
    {
        Instance crate = Box(crate_pos[k], crate_size, crate_mat, true);
        crate.yaw       = 0.35f * k;
        crate.grabbable = true; // serao os objetos da mecanica principal
        out.push_back(crate);
    }

    // Coelho e esfera do template, so para confirmar que malhas de OBJ
    // continuam entrando na cena pelo mesmo caminho das caixas.
    {
        Instance bunny;
        bunny.mesh      = "the_bunny";
        bunny.pos       = glm::vec3(-2.6f, 0.0f, 1.6f);
        bunny.yaw       = -0.6f;
        bunny.scale     = glm::vec3(1.0f);
        bunny.mat       = MakeMaterial(1, UV_PLANAR_XY, 1.0f, 0.15f, 20.0f);
        bunny.solid     = true;
        bunny.grabbable = false;
        out.push_back(bunny);

        Instance sphere;
        sphere.mesh      = "the_sphere";
        sphere.pos       = glm::vec3(2.4f, 0.6f, -1.9f);
        sphere.yaw       = 0.0f;
        sphere.scale     = glm::vec3(0.6f);
        sphere.mat       = MakeMaterial(1, UV_SPHERE, 1.0f, 0.35f, 48.0f);
        sphere.solid     = true;
        sphere.grabbable = false;
        out.push_back(sphere);
    }
}
