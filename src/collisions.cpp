#include "collisions.h"

#include <algorithm>
#include <cmath>
#include <limits>

AABB AABBTransform(const glm::vec3& local_min,
                   const glm::vec3& local_max,
                   const glm::mat4& model)
{
    const float BIG = std::numeric_limits<float>::max();

    AABB out;
    out.min = glm::vec3( BIG,  BIG,  BIG);
    out.max = glm::vec3(-BIG, -BIG, -BIG);

    // Percorremos os 8 cantos da caixa local, levamos cada um para mundo com a
    // Model matrix, e ficamos com a caixa envolvente dos resultados.
    for (int i = 0; i < 8; ++i)
    {
        glm::vec4 corner(
            (i & 1) ? local_max.x : local_min.x,
            (i & 2) ? local_max.y : local_min.y,
            (i & 4) ? local_max.z : local_min.z,
            1.0f);

        glm::vec4 w = model * corner;

        out.min.x = std::min(out.min.x, w.x);
        out.min.y = std::min(out.min.y, w.y);
        out.min.z = std::min(out.min.z, w.z);
        out.max.x = std::max(out.max.x, w.x);
        out.max.y = std::max(out.max.y, w.y);
        out.max.z = std::max(out.max.z, w.z);
    }

    return out;
}

AABB AABBExpand(const AABB& box, float margin)
{
    AABB out;
    out.min = box.min - glm::vec3(margin);
    out.max = box.max + glm::vec3(margin);
    return out;
}

glm::vec3 AABBCenter(const AABB& box)
{
    return (box.min + box.max) * 0.5f;
}

glm::vec3 AABBHalfSize(const AABB& box)
{
    return (box.max - box.min) * 0.5f;
}

float AABBHalfExtentAlongDir(const AABB& box, const glm::vec3& dir)
{
    // A projecao de uma caixa alinhada aos eixos sobre uma direcao qualquer
    // tem meia-extensao igual a soma das meias-arestas vezes o modulo da
    // componente correspondente da direcao.
    glm::vec3 h = AABBHalfSize(box);
    return h.x*std::fabs(dir.x) + h.y*std::fabs(dir.y) + h.z*std::fabs(dir.z);
}

bool AABBOverlap(const AABB& a, const AABB& b)
{
    // Duas caixas se intersectam somente se houver sobreposicao nos TRES eixos.
    if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
    if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
    if (a.max.z < b.min.z || a.min.z > b.max.z) return false;
    return true;
}

bool AABBContainsPoint(const AABB& box, const glm::vec3& p)
{
    return p.x >= box.min.x && p.x <= box.max.x
        && p.y >= box.min.y && p.y <= box.max.y
        && p.z >= box.min.z && p.z <= box.max.z;
}

bool RayAABB(const glm::vec3& origin, const glm::vec3& dir, const AABB& box,
             float* t_near, float* t_far, glm::vec3* normal)
{
    // Metodo das fatias: para cada eixo, o raio entra e sai de uma faixa
    // delimitada por dois planos. A intersecao com a caixa e a interseccao dos
    // tres intervalos. Se em algum momento o inicio passa do fim, nao ha
    // intersecao.
    const float EPS = 1e-8f;

    float tmin = 0.0f;
    float tmax = std::numeric_limits<float>::max();

    int   axis_min = -1;
    float sign_min = 1.0f;

    const float o[3] = { origin.x, origin.y, origin.z };
    const float d[3] = { dir.x,    dir.y,    dir.z    };
    const float lo[3] = { box.min.x, box.min.y, box.min.z };
    const float hi[3] = { box.max.x, box.max.y, box.max.z };

    for (int i = 0; i < 3; ++i)
    {
        if (std::fabs(d[i]) < EPS)
        {
            // Raio paralelo a esta fatia: so pode haver intersecao se a origem
            // ja estiver dentro dela.
            if (o[i] < lo[i] || o[i] > hi[i])
                return false;
            continue;
        }

        float inv = 1.0f / d[i];
        float t1  = (lo[i] - o[i]) * inv;
        float t2  = (hi[i] - o[i]) * inv;

        float s = -1.0f; // sinal da normal da face de entrada
        if (t1 > t2)
        {
            std::swap(t1, t2);
            s = 1.0f;
        }

        if (t1 > tmin)
        {
            tmin     = t1;
            axis_min = i;
            sign_min = s;
        }
        if (t2 < tmax)
            tmax = t2;

        if (tmin > tmax)
            return false;
    }

    if (t_near) *t_near = tmin;
    if (t_far)  *t_far  = tmax;

    if (normal)
    {
        *normal = glm::vec3(0.0f);
        if (axis_min == 0) normal->x = sign_min;
        else if (axis_min == 1) normal->y = sign_min;
        else if (axis_min == 2) normal->z = sign_min;
    }

    return true;
}

RayHit RaycastBoxes(const glm::vec3& origin, const glm::vec3& dir,
                    const std::vector<AABB>& boxes, int skip_index)
{
    RayHit best;
    best.hit    = false;
    best.t      = std::numeric_limits<float>::max();
    best.index  = -1;
    best.normal = glm::vec3(0.0f);

    for (size_t i = 0; i < boxes.size(); ++i)
    {
        if ((int)i == skip_index)
            continue;

        float tn, tf;
        glm::vec3 n;
        if (RayAABB(origin, dir, boxes[i], &tn, &tf, &n) && tn < best.t)
        {
            best.hit    = true;
            best.t      = tn;
            best.index  = (int)i;
            best.normal = n;
        }
    }

    if (!best.hit)
        best.t = 0.0f;

    return best;
}

bool AABBOverlapsAny(const AABB& box, const std::vector<AABB>& boxes,
                     int skip_index)
{
    for (size_t i = 0; i < boxes.size(); ++i)
    {
        if ((int)i == skip_index)
            continue;
        if (AABBOverlap(box, boxes[i]))
            return true;
    }
    return false;
}

int FindBoxContainingPoint(const glm::vec3& p,
                           const std::vector<AABB>& boxes,
                           int skip_index)
{
    for (size_t i = 0; i < boxes.size(); ++i)
    {
        if ((int)i == skip_index)
            continue;
        if (AABBContainsPoint(boxes[i], p))
            return (int)i;
    }
    return -1;
}

bool FindSupportHeight(const AABB& box,
                       const std::vector<AABB>& boxes,
                       int skip_index,
                       float tolerance,
                       float* out_height)
{
    // Mesma folga transversal de ResolveSingleAxis: caixas que so encostam
    // de lado (face com face) nao servem de apoio.
    const float EPS = 1e-4f;

    bool  found = false;
    float best  = 0.0f;

    for (size_t i = 0; i < boxes.size(); ++i)
    {
        if ((int)i == skip_index)
            continue;

        const AABB& b = boxes[i];

        if (box.max.x <= b.min.x + EPS || box.min.x >= b.max.x - EPS)
            continue;
        if (box.max.z <= b.min.z + EPS || box.min.z >= b.max.z - EPS)
            continue;

        // O topo da caixa candidata precisa estar abaixo da base do objeto.
        // A tolerancia aceita uma penetracao pequena (o pouso da mecanica
        // deixa uma folga de milimetros); sem ela, um objeto que encostasse
        // 1 mm dentro do piso o atravessaria, porque o piso nao estaria
        // "abaixo" dele.
        if (b.max.y > box.min.y + tolerance)
            continue;

        if (!found || b.max.y > best)
        {
            best  = b.max.y;
            found = true;
        }
    }

    if (found && out_height)
        *out_height = best;

    return found;
}

// Resolve o deslocamento da caixa "cur" ao longo de um unico eixo (0=X, 1=Y,
// 2=Z), truncando o passo caso alguma caixa solida da lista fique no caminho.
static float ResolveSingleAxis(AABB& cur, int axis, float move,
                               const std::vector<AABB>& boxes,
                               int skip_index, bool* out_hit)
{
    if (out_hit)
        *out_hit = false;

    if (move == 0.0f)
        return 0.0f;

    // Folga transversal: duas caixas que apenas se encostam face a face (por
    // exemplo, os pes do jogador apoiados em y = piso.max.y) NAO devem ser
    // tratadas como penetracao quando o movimento ocorre nos outros dois
    // eixos (X e Z). Caso contrario, o proprio piso travaria a caminhada.
    const float EPS = 1e-4f;

    const int u = (axis + 1) % 3;
    const int v = (axis + 2) % 3;

    float allowed = move;
    bool  hit     = false;

    for (size_t i = 0; i < boxes.size(); ++i)
    {
        if ((int)i == skip_index)
            continue;

        const AABB& b = boxes[i];

        // Precisa haver sobreposicao estrita nos dois eixos transversais.
        if (cur.max[u] <= b.min[u] + EPS || cur.min[u] >= b.max[u] - EPS)
            continue;
        if (cur.max[v] <= b.min[v] + EPS || cur.min[v] >= b.max[v] - EPS)
            continue;

        if (move > 0.0f)
        {
            // Obstaculo a frente no sentido positivo do eixo.
            if (cur.max[axis] <= b.min[axis] + EPS &&
                cur.max[axis] + allowed > b.min[axis])
            {
                allowed = b.min[axis] - cur.max[axis];
                if (allowed < 0.0f)
                    allowed = 0.0f;
                hit = true;
            }
        }
        else
        {
            // Obstaculo a frente no sentido negativo do eixo.
            if (cur.min[axis] >= b.max[axis] - EPS &&
                cur.min[axis] + allowed < b.max[axis])
            {
                allowed = b.max[axis] - cur.min[axis];
                if (allowed > 0.0f)
                    allowed = 0.0f;
                hit = true;
            }
        }
    }

    cur.min[axis] += allowed;
    cur.max[axis] += allowed;

    if (out_hit)
        *out_hit = hit;

    return allowed;
}

glm::vec3 ResolveAABBMovement(const AABB& box,
                              const glm::vec3& delta,
                              const std::vector<AABB>& boxes,
                              int skip_index,
                              bool out_hit_axis[3])
{
    if (out_hit_axis)
    {
        out_hit_axis[0] = false;
        out_hit_axis[1] = false;
        out_hit_axis[2] = false;
    }

    AABB      cur = box;
    glm::vec3 applied(0.0f);

    // Se o jogador esta subindo (pulo), resolvemos Y primeiro para que os pes
    // ganhem altura antes de avancar sobre a borda de um objeto; se esta
    // caindo ou caminhando no plano, resolvemos X e Z primeiro e apoiamos Y
    // por ultimo. A separacao por eixo garante deslizamento suave ao longo das
    // paredes sem travar nas quinas.
    const int order_up[3]   = { 1, 0, 2 };
    const int order_down[3] = { 0, 2, 1 };
    const int* order = (delta.y > 0.0f) ? order_up : order_down;

    for (int k = 0; k < 3; ++k)
    {
        int  axis = order[k];
        bool hit  = false;
        applied[axis] = ResolveSingleAxis(cur, axis, delta[axis],
                                          boxes, skip_index, &hit);
        if (out_hit_axis)
            out_hit_axis[axis] = hit;
    }

    return applied;
}
