#ifndef BEZIER_H
#define BEZIER_H

// Movimentacao de objeto por curva de Bezier cubica (requisito do enunciado e
// marco M3 do plano).
//
// Uma curva de Bezier cubica e definida por quatro pontos de controle
// P0, P1, P2, P3 e parametrizada por t em [0, 1]:
//
//     B(t) = (1-t)^3 * P0
//          + 3*(1-t)^2*t * P1
//          + 3*(1-t)*t^2 * P2
//          + t^3 * P3
//
// O parametro t avanca proporcionalmente ao tempo decorrido entre quadros (dt)
// e inverte o sentido nas extremidades (t = 0 e t = 1), produzindo um
// movimento continuo de ida e volta pela sala.

#include <glm/vec3.hpp>

struct BezierState
{
    int       instance;      // indice em g_Instances do objeto que percorre a curva
    glm::vec3 p0;            // ponto inicial (t = 0)
    glm::vec3 p1;            // primeiro ponto de controle intermediario
    glm::vec3 p2;            // segundo ponto de controle intermediario
    glm::vec3 p3;            // ponto final (t = 1)
    float     t;             // parametro atual em [0, 1]
    float     direction;     // +1.0f na ida (0 -> 1), -1.0f na volta (1 -> 0)
    float     speed;         // variacao de t por segundo (1 / duracao da travessia)
    float     angular_speed; // velocidade de rotacao propria em torno de Y (rad/s)
};

// Avalia o ponto B(t) da curva de Bezier cubica para t em [0, 1].
glm::vec3 BezierCubicPoint(const glm::vec3& p0,
                           const glm::vec3& p1,
                           const glm::vec3& p2,
                           const glm::vec3& p3,
                           float t);

// Avalia a derivada dB/dt(t) (vetor tangente a curva) para t em [0, 1].
glm::vec3 BezierCubicTangent(const glm::vec3& p0,
                             const glm::vec3& p1,
                             const glm::vec3& p2,
                             const glm::vec3& p3,
                             float t);

// Inicializa a curva de Bezier cubica associada a instancia "instance_index".
void BezierInit(BezierState& b, int instance_index);

// Atualiza o parametro t pelo passo de tempo "dt" (em segundos) e reposiciona
// a instancia correspondente em g_Instances.
void BezierUpdate(BezierState& b, float dt);

#endif // BEZIER_H
