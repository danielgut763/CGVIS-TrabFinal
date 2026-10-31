#ifndef GRAB_H
#define GRAB_H

// Mecanica principal do jogo: perspectiva forcada.
//
// Enquanto um objeto esta sendo carregado, ele mantem exatamente o mesmo
// TAMANHO APARENTE na tela. Ao ser solto, ele e reposicionado onde o raio da
// camera encontra a cena e escalado proporcionalmente a essa distancia, o que
// faz um objeto pequeno e perto virar um objeto grande e longe.
//
// A matematica esta derivada em detalhe nos comentarios de grab.cpp. Em
// resumo, sob projecao perspectiva a altura projetada de um objeto cai com
// 1/z, entao manter o tamanho aparente constante exige escala proporcional a
// distancia: s(d) = s0 * (d/d0).
//
// Este modulo nao usa OpenGL. Ele le e escreve g_Instances (scene.h) e
// consulta a cena atraves das AABBs recebidas por parametro.

#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "camera.h"
#include "collisions.h"
#include "scene.h"

namespace GrabConfig
{
    const float MIN_DISTANCE  = 0.70f; // o objeto nunca encosta na camera
    const float MIN_FACTOR    = 0.20f; // escala minima, relativa a do momento de pegar
    const float MAX_FACTOR    = 6.00f; // escala maxima, relativa a do momento de pegar
    const float ROTATE_SPEED  = 2.20f; // rad/s ao girar o objeto com Q e E
    const float REACH_STEP    = 0.07f; // quanto a roda do mouse altera o alcance
    const float MIN_REACH     = 0.12f;
    const float PLACE_MARGIN  = 0.004f; // folga deixada entre o objeto e a cena
    const int   REFINE_STEPS  = 14;     // iteracoes da busca binaria de pouso
}

struct GrabState
{
    int   instance;    // indice em g_Instances do objeto carregado (-1 = nenhum)

    // --- Estado congelado no instante em que o objeto foi pego ---
    glm::vec3 dir_local; // direcao camera->centro do objeto, nas coordenadas da
                         // base da camera (right, up, forward). Normalizada.
                         // E o que prende o objeto ao mesmo ponto da tela.
    float d0;            // distancia camera->centro naquele instante
    float s0;            // escala uniforme do objeto naquele instante
    float r0;            // meia-extensao do objeto ao longo do raio, em d0
    float yaw0;          // rotacao original do objeto
    bool  was_solid;     // se o objeto era solido antes de ser pego

    // --- Estado controlado pelo jogador enquanto carrega ---
    float yaw_offset;    // rotacao acumulada com Q e E
    float reach;         // fracao do alcance maximo, ajustada pela roda (0..1]

    // --- Resultado do ultimo quadro, util para HUD e depuracao ---
    float cur_distance;
    float cur_factor;    // escala atual dividida por s0
    bool  blocked;       // verdadeiro se o objeto nao cabe onde esta mirando
};

// Implementada em main.cpp: bounding box LOCAL da malha (vem de g_VirtualScene,
// preenchida pelo carregador de OBJ). Declarada aqui para que este modulo nao
// precise conhecer OpenGL.
bool GetMeshLocalBBox(const std::string& mesh, glm::vec3* out_min, glm::vec3* out_max);

void GrabInit(GrabState& g);

inline bool GrabIsHolding(const GrabState& g) { return g.instance >= 0; }

// Constroi o raio que sai da camera e passa pelo ponto (ndc_x, ndc_y) da tela,
// ambos em [-1, 1]. Usa a base ortonormal da camera, SEM inverter matriz e sem
// gluUnProject/glm::unProject, que sao proibidas pelo enunciado.
void BuildCursorRay(const Camera& cam, float screen_ratio,
                    float ndc_x, float ndc_y,
                    glm::vec3* out_origin, glm::vec3* out_dir);

// Tenta pegar o objeto sob o cursor. "boxes" sao as AABBs solidas da cena e
// "box_instance" diz a qual instancia cada caixa pertence. Devolve true se
// algum objeto marcado como grabbable foi pego.
bool GrabTryPick(GrabState& g, Camera& cam, float screen_ratio,
                 float ndc_x, float ndc_y,
                 const std::vector<AABB>& boxes,
                 const std::vector<int>& box_instance);

// Recalcula a cada quadro a posicao e a escala do objeto carregado.
void GrabUpdate(GrabState& g, const Camera& cam,
                const std::vector<AABB>& boxes,
                const std::vector<int>& box_instance,
                const AABB& player_box);

// Solta o objeto na posicao atual. Devolve false (e continua segurando) se o
// objeto nao couber ali sem atravessar a cena ou o proprio jogador.
bool GrabTryRelease(GrabState& g, Camera& cam,
                    const std::vector<AABB>& boxes,
                    const std::vector<int>& box_instance,
                    const AABB& player_box);

// Rotacao do objeto na mao (teclas Q e E), em radianos.
void GrabRotate(GrabState& g, float delta_yaw);

// Ajuste do alcance pela roda do mouse: aproxima ou afasta o objeto, o que
// por consequencia o deixa menor ou maior.
void GrabAdjustReach(GrabState& g, float wheel);

#endif // GRAB_H
