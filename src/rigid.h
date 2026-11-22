#ifndef RIGID_H
#define RIGID_H

// Orientacao livre dos objetos pegaveis (M8: tombar e rolar).
//
// Ate o M7 todo objeto so girava em torno de Y (yaw). Para tombar e rolar, o
// objeto ganha uma segunda rotacao, Instance::tilt, e a Model matrix passa a
// ser T * Ry(yaw) * tilt * S (scene.cpp).
//
// Este modulo junta a matematica de rotacao que os outros usam (tip.cpp,
// roll.cpp, physics.cpp, grab.cpp). Tudo escrito a mao: Rodrigues e
// Gram-Schmidt, sem helper de rotacao da GLM (vetado pelo enunciado).
// Matrix_Rotate de matrices.h (nossa, do laboratorio) pode ser usada.
//
// Convencoes:
//   - "orientacao" de uma instancia = Ry(yaw) * tilt (so a parte 3x3 importa;
//     a quarta coluna de tilt e sempre (0,0,0,1)).
//   - eixos e angulos em MUNDO, salvo quando o nome diz "local".
//   - angulo positivo = regra da mao direita em torno do eixo.
//   - "centro" de uma instancia = centro da bbox local da malha levado para
//     mundo, que e o centro de massa que a fisica usa.

#include <vector>

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

#include "collisions.h"
#include "scene.h"

// ---------------------------------------------------------------------------
// Tipos compartilhados entre physics.cpp, tip.cpp e roll.cpp.
// ---------------------------------------------------------------------------

// O que physics.cpp sabe sobre a vizinhanca de um objeto neste passo.
struct PhysicsContext
{
    std::vector<AABB>*      boxes;    // caixas solidas da cena + jogador (a ULTIMA)
    const std::vector<int>* owner;    // instancia dona de cada caixa (-1 = jogador)
    int       self;                   // indice da caixa DESTE objeto em *boxes
    int       instance;               // indice dele em g_Instances
    glm::vec3 lmin, lmax;             // bbox local da malha
};

// Resultado do teste de apoio/equilibrio de um objeto (preenchido por
// physics.cpp com FindSupportHeight + SupportFootprint).
struct SupportInfo
{
    bool      supported;      // ha algo logo abaixo (dentro de SUPPORT_TOL)
    float     height;         // y do topo do apoio
    int       box_index;      // caixa de apoio em *boxes (-1 se nenhuma)
    float     fx0, fx1;       // area de apoio no plano XZ (SupportFootprint)
    float     fz0, fz1;
    bool      unbalanced;     // centro de massa fora da area (com BALANCE_MARGIN)
    glm::vec3 overhang;       // vetor HORIZONTAL do ponto mais proximo da area
                              // ate o centro de massa (zero se equilibrado)
};

// ---------------------------------------------------------------------------
// Rotacoes.
// ---------------------------------------------------------------------------

// Matriz de rotacao (Rodrigues) de "angle" radianos em torno do eixo
// unitario "axis". Quarta coluna/linha de identidade.
glm::mat4 RotationAboutAxis(const glm::vec3& axis, float angle);

// Gram-Schmidt nas tres primeiras colunas, para matar o acumulo de erro de
// ponto flutuante de uma rotacao composta milhares de vezes (bola rolando).
void OrthonormalizeRotation(glm::mat4& R);

// Ry(yaw) * tilt.
glm::mat4 InstanceOrientation(const Instance& inst);

// Aplica uma rotacao em MUNDO a orientacao da instancia, mantendo o yaw e
// mudando so tilt:  Ry(yaw)*tilt  <-  Rot(axis, angle) * Ry(yaw)*tilt.
// NAO mexe na posicao (a origem da malha fica parada; ver as funcoes abaixo
// para girar em torno do centro ou de uma reta).
void ApplyWorldRotation(Instance& inst, const glm::vec3& axis, float angle);

// Centro (da bbox local) em mundo, e a posicao inversa: ajusta inst.pos para
// que o centro caia em "center", com a orientacao e escala atuais.
glm::vec3 InstanceBodyCenter(const Instance& inst, const glm::vec3& lmin, const glm::vec3& lmax);
void      SetInstanceBodyCenter(Instance& inst, const glm::vec3& lmin, const glm::vec3& lmax,
                                const glm::vec3& center);

// Gira a instancia em torno do proprio centro (orientacao muda, centro fica).
void RotateAboutCenter(Instance& inst, const glm::vec3& lmin, const glm::vec3& lmax,
                       const glm::vec3& axis, float angle);

// Gira a instancia em torno da reta que passa por "pivot" com direcao "axis"
// (unitario): o centro descreve um arco em torno da reta e a orientacao gira
// junto. E o movimento de um objeto tombando sobre uma aresta.
void RotateAboutLine(Instance& inst, const glm::vec3& lmin, const glm::vec3& lmax,
                     const glm::vec3& pivot, const glm::vec3& axis, float angle);

// ---------------------------------------------------------------------------
// Poses estaveis, por forma (BodyShape).
//   SHAPE_BOX:    algum dos 6 eixos locais (+-X, +-Y, +-Z) aponta para +Y.
//   SHAPE_SPHERE: qualquer pose.
//   SHAPE_PAWN:   em pe (+Y local para +Y) ou deitado (Y local na
//                 horizontal, qualquer rotacao em torno do proprio eixo).
//                 De cabeca para baixo NAO e estavel: vai para deitado.
// ---------------------------------------------------------------------------

// Menor rotacao em MUNDO (eixo unitario + angulo >= 0) que leva a orientacao
// atual para a pose estavel mais proxima. Devolve false (e angulo 0) se ja
// esta estavel dentro de "tolerance" radianos.
bool NearestStablePose(const Instance& inst, float tolerance,
                       glm::vec3* out_axis, float* out_angle);

// Depois de assentar: reescreve yaw e tilt para a forma canonica, sem mudar
// a orientacao visivel alem do erro numerico. Para SHAPE_BOX, tilt vira
// exatamente uma das 24 rotacoes do cubo (entradas 0, +1, -1) e o resto da
// rotacao em torno de Y vai para o yaw. Para SHAPE_PAWN em pe, tilt vira a
// identidade. Para os demais, so ortonormaliza.
void CanonicalizePose(Instance& inst);

// Peao deitado: eixo local Y na horizontal (|y do eixo em mundo| < 0.5).
bool PawnIsLying(const Instance& inst);

// Direcao em MUNDO do eixo local Y (eixo de revolucao do peao).
glm::vec3 InstanceLocalAxisWorld(const Instance& inst, int local_axis /*0=X,1=Y,2=Z*/);

// ---------------------------------------------------------------------------
// Caixa de colisao de objetos inclinados.
//
// A AABB pelos 8 cantos rotacionados (AABBTransform) e correta para caixas,
// mas para corpos redondos ela incha: um peao deitado girando em torno do
// proprio eixo ganharia ate 41% de altura e ficaria flutuando. Por isso:
//   SHAPE_SPHERE: centro +- raio (raio = metade do maior lado da bbox local
//                 vezes a escala).
//   SHAPE_PAWN:   AABB exata de um cilindro com o eixo de revolucao: por eixo
//                 de mundo i, meia-extensao = |u_i|*h + r*sqrt(1 - u_i^2),
//                 com u o eixo em mundo, h a meia-altura e r o raio da bbox.
//   SHAPE_BOX:    AABBTransform normal.
// InstanceCollisionBox (scene.cpp) chama esta funcao para pegaveis.
// ---------------------------------------------------------------------------
AABB BodyWorldAABB(const Instance& inst, const glm::vec3& lmin, const glm::vec3& lmax);

#endif // RIGID_H
