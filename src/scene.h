#ifndef SCENE_H
#define SCENE_H

// Descricao da cena como uma LISTA DE INSTANCIAS. Cada instancia aponta para
// uma malha ja carregada em g_VirtualScene (main.cpp) e carrega a sua propria
// Model matrix e o seu material. Com isso, a mesma malha pode aparecer varias
// vezes na cena com transformacoes diferentes, que e o requisito de
// "instancias de objetos" do enunciado.

#include <string>
#include <vector>

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

// Como as coordenadas de textura sao obtidas no fragment shader.
// Precisa bater com as constantes de shader_fragment.glsl.
enum UVMode
{
    UV_OBJ       = 0, // usa as coordenadas vindas do arquivo OBJ
    UV_WORLD     = 1, // projecao planar em COORDENADAS DE MUNDO, pelo eixo
                      // dominante da normal. Mantem o texel quadrado em
                      // qualquer parede, independente da escala: e isto que
                      // evita o desconto por "textura esticada".
    UV_SPHERE    = 2, // projecao esferica em coordenadas do modelo
    UV_PLANAR_XY = 3  // projecao planar XY em coordenadas do modelo
};

struct Material
{
    int   texture_id; // qual TextureImageN usar (0, 1, 2, ...)
    int   uv_mode;    // UVMode
    float uv_scale;   // SO para UV_WORLD: metros de mundo por repeticao da textura
    float ks;         // intensidade do termo especular (Blinn-Phong)
    float shininess;  // expoente do especular
};

Material MakeMaterial(int texture_id, int uv_mode, float uv_scale,
                      float ks = 0.1f, float shininess = 24.0f);

struct Instance
{
    std::string mesh;      // chave em g_VirtualScene, ex: "the_cube"
    glm::vec3   pos;       // posicao do CENTRO em coordenadas de mundo
    float       yaw;       // rotacao em torno de Y, em radianos
    glm::vec3   scale;     // escala por eixo (paredes usam escala nao uniforme)
    Material    mat;

    bool solid;            // participa dos testes de colisao
    bool grabbable;        // o jogador pode pegar e escalar (mecanica principal)
};

// Model matrix da instancia: T * Ry * S, aplicada da direita para a esquerda.
glm::mat4 InstanceModelMatrix(const Instance& inst);

// Lista global dos objetos desenhados a cada quadro.
extern std::vector<Instance> g_Instances;

// Indice em g_Instances do objeto decorativo flutuante que percorre a curva
// de Bezier cubica (definido em BuildScene).
extern int g_BezierInstance;

// ---------------------------------------------------------------------------
// Dimensoes da fase, em metros. Ficam aqui para que colisao, jogador e camera
// usem os MESMOS numeros que a geometria desenhada.
// ---------------------------------------------------------------------------
namespace Room
{
    const float HALF_X    = 4.0f;  // interior vai de -4 a +4 em X
    const float HALF_Z    = 3.0f;  // interior vai de -3 a +3 em Z
    const float HEIGHT    = 4.0f;  // piso em y=0, teto em y=4 (deixa 2 m de altura livre no vao)
    const float THICKNESS = 0.2f;  // espessura de parede, piso e teto

    // Saida elevada, aberta na parede +X. Comeca acima da altura do jogador,
    // entao so da para alcancar subindo em algum objeto aumentado.
    const float EXIT_Y0    = 2.0f; // base do vao
    const float EXIT_WIDTH = 1.2f; // largura do vao, centrado em z = 0
}

// Preenche a lista com piso, teto, as quatro paredes (a de +X dividida em
// tres pedacos para deixar o vao da saida) e alguns objetos de teste.
void BuildScene(std::vector<Instance>& out);

#endif // SCENE_H
