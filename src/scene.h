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
    PATTERN_CEILING = 3, // teto claro dividido em paineis
    PATTERN_DOOR    = 4, // porta metalica da saida (M6)
    PATTERN_BUTTON_IDLE    = 5, // botao solto: vermelho (M6)
    PATTERN_BUTTON_LIGHT   = 6, // apertado por algo leve demais: ambar
    PATTERN_BUTTON_ACTIVE  = 7  // apertado por objeto grande: verde, porta abre
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

// Forma fisica de um objeto pegavel (M8: tombar e rolar). Decide quais poses
// sao estaveis e se o objeto rola.
enum BodyShape
{
    SHAPE_BOX    = 0, // dado, cubo magico, tijolo, coelho: tomba de face em
                      // face (poses estaveis: algum eixo local na vertical)
    SHAPE_SPHERE = 1, // bola: qualquer pose e estavel, rola sem deslizar
    SHAPE_PAWN   = 2  // corpo de revolucao em torno do Y local, com base
                      // plana em -Y: em pe e estavel; deitado (Y local na
                      // horizontal) rola em torno do proprio eixo
};

// Estado de movimento de um objeto pegavel solto (M8, ver physics.h).
enum MotionState
{
    MOTION_REST   = 0, // apoiado, parado (ou dormindo)
    MOTION_FALL   = 1, // no ar: cai e gira com ang_vel em torno do centro
    MOTION_TIP    = 2, // tombando: gira em torno de uma aresta do apoio
    MOTION_SETTLE = 3, // acabou de pousar torto: assentando numa pose estavel
    MOTION_ROLL   = 4  // rolando sobre o apoio (bola, peao deitado)
};

// Estado fisico de rotacao (M8). Zerado quando o objeto e pego.
struct BodyState
{
    int       motion  = MOTION_REST;
    glm::vec3 ang_vel = glm::vec3(0.0f); // velocidade angular em MUNDO, rad/s
                                         // (direcao = eixo, modulo = rapidez)
    glm::vec3 pivot   = glm::vec3(0.0f); // TIP: ponto da aresta de apoio (mundo)
    glm::vec3 axis    = glm::vec3(0.0f); // TIP/SETTLE: eixo unitario em mundo
    float     angle   = 0.0f;            // TIP: angulo ja girado; SETTLE: angulo que falta
    float     timer   = 0.0f;            // tempo no estado atual (s)
    float     sleep   = 0.0f;            // tempo parado; acima de SLEEP_TIME dorme
    int       support = -1;              // instancia que apoiava no ultimo passo (-1 = nenhuma/cenario)
};

struct Instance
{
    std::string mesh;      // chave em g_VirtualScene, ex: "the_cube"
    glm::vec3   pos;       // posicao do CENTRO em coordenadas de mundo
    float       yaw;       // rotacao em torno de Y, em radianos
    glm::vec3   scale;     // escala por eixo (paredes usam escala nao uniforme)

    // Inclinacao do objeto (M8): rotacao aplicada ANTES do yaw, no referencial
    // do objeto ja girado pelo yaw. Identidade para tudo que nunca tombou.
    // Model = T * Ry(yaw) * tilt * S. Assim Q/E (yaw) continua girando o
    // objeto em torno do Y do MUNDO mesmo depois de ele ter tombado.
    glm::mat4   tilt = glm::mat4(1.0f);

    int         shape = SHAPE_BOX;  // BodyShape (so importa para pegaveis)
    BodyState   body;
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

    // Velocidade em m/s dos objetos pegaveis quando estao soltos (M6/M7, ver
    // physics.h): queda em Y e deslizamento em XZ quando o objeto perde o
    // equilibrio. Zero quando apoiado ou na mao.
    glm::vec3 vel = glm::vec3(0.0f);
};

// Model matrix da instancia: T * Ry(yaw) * tilt * S, aplicada da direita para
// a esquerda.
glm::mat4 InstanceModelMatrix(const Instance& inst);

// AABB em mundo usada nos TESTES DE COLISAO da instancia de indice "idx",
// a partir da bounding box local da malha. E o unico lugar que sabe que a
// porta e desenhada com 0,10 m de espessura mas colide com os 0,20 m da
// parede (ver scene.cpp). Tudo que monta lista de caixas (main.cpp,
// physics.cpp, level.cpp) passa por aqui, para nunca discordar.
struct AABB;
AABB InstanceCollisionBox(int idx, const glm::vec3& local_min, const glm::vec3& local_max);

// Mesma coisa para uma copia da instancia em outra posicao (a porta sendo
// testada antes de se mover, em level.cpp). "idx" diz qual instancia ela e.
AABB InstanceCollisionBox(const Instance& inst, int idx,
                          const glm::vec3& local_min, const glm::vec3& local_max);

// Lista global dos objetos desenhados a cada quadro.
extern std::vector<Instance> g_Instances;

// Indice em g_Instances do objeto decorativo flutuante que percorre a curva
// de Bezier cubica (definido em BuildScene).
extern int g_BezierInstance;

// Indices em g_Instances das pecas do puzzle (M6), definidos em BuildScene.
extern int g_ButtonInstance;   // botao no piso
extern int g_DoorInstance;     // porta que fecha a saida elevada

// Extremos da luminaria curta no centro do teto, definidos em BuildScene. A
// luz 1 e uma fonte PONTUAL no ponto medio entre eles; e essa posicao que o
// shader e o cube map de sombra usam.
extern glm::vec3 g_LampP0;
extern glm::vec3 g_LampP1;

// Posicao da segunda luminaria: o pendente em cone na quina oposta a porta.
// E um SPOTLIGHT apontado para baixo, com queda mais rapida que a luz
// central, e serve de luz de apoio para aquele canto da sala.
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

// ---------------------------------------------------------------------------
// Pecas do puzzle da fase (M6), em metros.
//
// A ideia, como descrita nas limitacoes do SPEC.md: um objeto grande sobre o
// botao abre a porta da saida elevada, e esse mesmo objeto (com mais um menor
// servindo de degrau) e a escada para chegar ate ela.
// ---------------------------------------------------------------------------
namespace Puzzle
{
    // Botao no piso, a frente da saida. Nao e solido: e uma placa fina que
    // afunda quando algo esta em cima, sem virar um degrau que travaria o
    // jogador ao caminhar.
    const float BUTTON_X      = 4.5f;
    const float BUTTON_Z      = 0.0f;
    const float BUTTON_SIZE   = 1.0f;   // lado da placa
    const float BUTTON_UP_H   = 0.05f;  // altura solto
    const float BUTTON_DOWN_H = 0.008f; // altura apertado

    // Altura minima do objeto sobre o botao para abrir a porta. E o "peso
    // suficiente" do jogo de referencia: so um objeto aumentado pela
    // mecanica passa disso.
    const float MIN_ACTIVATE_HEIGHT = 1.0f;

    // Porta: placa que desliza para BAIXO, para dentro da parede abaixo do
    // vao. Fica no meio da espessura da parede, sem faces coplanares, e ao
    // abrir para 1 cm abaixo do peitoril (2,20 - 2,01 = 0,19 m na base e
    // 4,20 - 2,01 = 2,19 m no topo) para nao deixar degrau na passagem.
    const float DOOR_THICKNESS = 0.10f;
    const float DOOR_TRAVEL    = 2.01f; // para 1 cm abaixo do peitoril
    const float DOOR_SPEED     = 0.9f;  // m/s

    // Corredor do lado de fora da saida; chegar ao fundo dele vence a fase.
    const float CORRIDOR_LENGTH = 1.2f;
}

// Caixa que envolve toda a geometria da fase, incluindo o patamar externo da
// saida. O mapa de sombra ajusta o seu volume de projecao a estes limites,
// para cobrir a sala inteira sem sobrar nem faltar resolucao.
void SceneWorldBounds(glm::vec3* out_min, glm::vec3* out_max);

// Preenche a lista com piso, teto, as quatro paredes (a de +X dividida em
// tres pedacos para deixar o vao da saida) e os objetos da fase.
void BuildScene(std::vector<Instance>& out);

#endif // SCENE_H
