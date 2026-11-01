#include "bezier.h"
#include "scene.h"

glm::vec3 BezierCubicPoint(const glm::vec3& p0,
                           const glm::vec3& p1,
                           const glm::vec3& p2,
                           const glm::vec3& p3,
                           float t)
{
    // Polinomios de Bernstein de grau 3:
    //   b0(t) = (1 - t)^3
    //   b1(t) = 3 * (1 - t)^2 * t
    //   b2(t) = 3 * (1 - t) * t^2
    //   b3(t) = t^3
    float u  = 1.0f - t;
    float u2 = u * u;
    float t2 = t * t;

    float b0 = u2 * u;
    float b1 = 3.0f * u2 * t;
    float b2 = 3.0f * u  * t2;
    float b3 = t2 * t;

    return b0 * p0 + b1 * p1 + b2 * p2 + b3 * p3;
}

glm::vec3 BezierCubicTangent(const glm::vec3& p0,
                             const glm::vec3& p1,
                             const glm::vec3& p2,
                             const glm::vec3& p3,
                             float t)
{
    // Derivada primeira da curva cubica em relacao a t:
    //   B'(t) = 3*(1-t)^2*(P1 - P0) + 6*(1-t)*t*(P2 - P1) + 3*t^2*(P3 - P2)
    float u = 1.0f - t;

    return (3.0f * u * u) * (p1 - p0)
         + (6.0f * u * t) * (p2 - p1)
         + (3.0f * t * t) * (p3 - p2);
}

void BezierInit(BezierState& b, int instance_index)
{
    b.instance = instance_index;

    // Trajetoria tridimensional em "S" na metade superior da sala, visivel
    // logo a frente da posicao inicial do jogador e acima da area de
    // caminhada para funcionar como objeto flutuante decorativo (SPEC.md).
    b.p0 = glm::vec3(-4.2f, 2.6f, -3.2f);
    b.p1 = glm::vec3( 1.5f, 3.6f, -1.6f);
    b.p2 = glm::vec3(-1.5f, 2.0f,  1.6f);
    b.p3 = glm::vec3( 4.2f, 3.0f,  3.2f);

    b.t             = 0.0f;
    b.direction     = 1.0f;
    b.speed         = 0.22f; // ~4.5 segundos para percorrer de uma ponta a outra
    b.angular_speed = 1.10f; // rotacao suave em torno de Y

    if (b.instance >= 0 && b.instance < (int)g_Instances.size())
    {
        g_Instances[b.instance].pos = BezierCubicPoint(b.p0, b.p1, b.p2, b.p3, b.t);
    }
}

void BezierUpdate(BezierState& b, float dt)
{
    if (dt <= 0.0f)
        return;

    if (b.instance < 0 || b.instance >= (int)g_Instances.size())
        return;

    // Avanca o parametro t proporcionalmente a dt, invertendo o sentido ao
    // atingir os extremos [0, 1] (movimento continuo de ida e volta).
    b.t += b.direction * b.speed * dt;

    if (b.t >= 1.0f)
    {
        b.t         = 2.0f - b.t;
        b.direction = -1.0f;
        if (b.t < 0.0f)
            b.t = 0.0f;
    }
    else if (b.t <= 0.0f)
    {
        b.t         = -b.t;
        b.direction = 1.0f;
        if (b.t > 1.0f)
            b.t = 1.0f;
    }

    Instance& inst = g_Instances[b.instance];
    inst.pos  = BezierCubicPoint(b.p0, b.p1, b.p2, b.p3, b.t);
    inst.yaw += b.angular_speed * dt;
}
