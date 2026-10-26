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
