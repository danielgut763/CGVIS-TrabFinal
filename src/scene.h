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

// Padrao procedural usado para gerar a cor do objeto. O SPEC.md especifica
// que "os objetos em cena serao na sua grande maioria composicoes de texturas
// procedurais, para emular as paredes, o chao e os objetos", entao parede,
// piso e teto sao desenhados por formula no fragment shader em vez de vir de
// uma imagem. Precisa bater com as constantes de shader_fragment.glsl.
enum PatternMode
{
    PATTERN_TEXTURE = 0, // cor lida de uma imagem (TextureImageN)
    PATTERN_WALL    = 1, // paineis creme com juntas verticais e rodape verde
    PATTERN_FLOOR   = 2, // piso verde com grade
    PATTERN_CEILING = 3  // teto claro dividido em paineis
};

// Modelo de interpolacao de iluminacao (M5 - Assets e luz).
// Precisa bater com as constantes de shader_vertex.glsl e shader_fragment.glsl.
enum ShadingMode
{
    SHADING_PHONG   = 0, // Blinn-Phong avaliado por fragmento (interpolacao de Phong)
    SHADING_GOURAUD = 1  // Blinn-Phong avaliado por vertice (interpolacao de Gouraud)
};

struct Material
{
    int   texture_id; // qual TextureImageN usar (so com PATTERN_TEXTURE)
    int   uv_mode;    // UVMode
    float uv_scale;   // SO para UV_WORLD: metros de mundo por repeticao da textura
    float ks;         // intensidade do termo especular (Blinn-Phong)
    float shininess;  // expoente do especular
    int   pattern;    // PatternMode
    float emissive;   // > 0 faz o objeto brilhar com cor propria, ignorando a
                      // iluminacao e a sombra. Usado na luminaria do teto.
    int   shading;    // ShadingMode (SHADING_PHONG ou SHADING_GOURAUD)
};

Material MakeMaterial(int texture_id, int uv_mode, float uv_scale,
                      float ks = 0.1f, float shininess = 24.0f,
                      int pattern = PATTERN_TEXTURE, float emissive = 0.0f,
                      int shading = SHADING_PHONG);

struct Instance
{
    std::string mesh;      // chave em g_VirtualScene, ex: "the_cube"
    glm::vec3   pos;       // posicao do CENTRO em coordenadas de mundo
    float       yaw;       // rotacao em torno de Y, em radianos
    glm::vec3   scale;     // escala por eixo (paredes usam escala nao uniforme)
    Material    mat;

    bool solid;            // participa dos testes de colisao
    bool grabbable;        // o jogador pode pegar e escalar (mecanica principal)

    // Se entra no passo de profundidade do shadow mapping.
    //
    // A casca da sala (piso, teto, paredes) e as luminarias NAO projetam. A
    // luz e direcional e vem de cima: se o teto projetasse, ele sozinho
    // cobriria a sala inteira e tudo ficaria na sombra. E as paredes, com a
    // luz inclinada, jogariam uma faixa escura de quase 2 m no chao ao longo
    // de dois lados, o que nao corresponde a um interior iluminado por
    // paineis no teto.
    //
    // Elas continuam RECEBENDO sombra normalmente: o que muda e so quem
    // bloqueia a luz. Na pratica, sao os objetos da fase que projetam, que e
    // exatamente o que interessa para o jogador perceber o tamanho deles.
    bool casts_shadow;
};

// Model matrix da instancia: T * Ry * S, aplicada da direita para a esquerda.
glm::mat4 InstanceModelMatrix(const Instance& inst);

// Lista global dos objetos desenhados a cada quadro.
extern std::vector<Instance> g_Instances;

// Indice em g_Instances do objeto decorativo flutuante que percorre a curva
// de Bezier cubica (definido em BuildScene).
extern int g_BezierInstance;

// Extremos do segmento que representa a luminaria em faixa, definidos em
// BuildScene.
//
// A iluminacao NAO usa uma direcao fixa nem um ponto unico: para cada
// fragmento ela procura o ponto mais proximo DESTE SEGMENTO. E isso que faz a
// luz chegar na vertical para quem esta embaixo da faixa e inclinada para
// quem esta ao lado dela, que e como uma luminaria comprida se comporta de
// verdade. Com uma luz direcional, todo ponto da sala recebia luz com a mesma
// inclinacao, inclusive os que estao exatamente sob a luminaria.
extern glm::vec3 g_LampP0;
extern glm::vec3 g_LampP1;

// Posicao da segunda luminaria: o pendente em cone na quina oposta a porta.
// E uma fonte PONTUAL, com queda mais rapida que a faixa central, e serve de
// luz de apoio para aquele canto da sala.
extern glm::vec3 g_Lamp2Pos;

// ---------------------------------------------------------------------------
// Dimensoes da fase, em metros. Ficam aqui para que colisao, jogador e camera
// usem os MESMOS numeros que a geometria desenhada.
// ---------------------------------------------------------------------------
namespace Room
{
    const float HALF_X    = 6.0f;  // interior vai de -6 a +6 em X
    const float HALF_Z    = 4.5f;  // interior vai de -4.5 a +4.5 em Z
    const float HEIGHT    = 4.2f;  // piso em y=0, teto em y=4.2
    const float THICKNESS = 0.2f;  // espessura de parede, piso e teto

    // Saida elevada, aberta na parede +X. Comeca acima da altura do jogador,
    // entao so da para alcancar subindo em algum objeto aumentado.
    const float EXIT_Y0    = 2.2f; // base do vao
    const float EXIT_WIDTH = 1.5f; // largura do vao, centrado em z = 0
}

// Caixa que envolve toda a geometria da fase, incluindo o patamar externo da
// saida. O mapa de sombra ajusta o seu volume de projecao a estes limites,
// para cobrir a sala inteira sem sobrar nem faltar resolucao.
void SceneWorldBounds(glm::vec3* out_min, glm::vec3* out_max);

// Preenche a lista com piso, teto, as quatro paredes (a de +X dividida em
// tres pedacos para deixar o vao da saida) e os objetos da fase.
void BuildScene(std::vector<Instance>& out);

#endif // SCENE_H
