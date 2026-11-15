#include "physics.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "grab.h"   // GetMeshLocalBBox
#include "scene.h"

void UpdateFallingObjects(int held_instance, const AABB& player_box, float dt)
{
    if (dt <= 0.0f)
        return;

    // -----------------------------------------------------------------------
    // 1) AABBs em mundo de tudo que e solido, mais o jogador.
    //
    // A lista e montada uma vez por quadro e atualizada a cada objeto movido,
    // para que dois objetos caindo um sobre o outro no mesmo quadro se vejam
    // nas posicoes corretas.
    // -----------------------------------------------------------------------
    std::vector<AABB> boxes;
    std::vector<int>  owner;   // instancia dona de cada caixa (-1 = jogador)
    boxes.reserve(g_Instances.size() + 1);
    owner.reserve(g_Instances.size() + 1);

    for (size_t i = 0; i < g_Instances.size(); ++i)
    {
        const Instance& inst = g_Instances[i];
        if (!inst.solid)
            continue;

        glm::vec3 lmin, lmax;
        if (!GetMeshLocalBBox(inst.mesh, &lmin, &lmax))
            continue;

        AABB b = AABBTransform(lmin, lmax, InstanceModelMatrix(inst));
        if ((int)i == g_DoorInstance)
        {
            b.min.x = Room::HALF_X;
            b.max.x = Room::HALF_X + Room::THICKNESS;
        }
        boxes.push_back(b);
        owner.push_back((int)i);
    }

    const int player_k = (int)boxes.size();
    boxes.push_back(player_box);
    owner.push_back(-1);

    // -----------------------------------------------------------------------
    // 2) Queda de cada objeto pegavel, ordenados do mais baixo para o mais
    //    alto (min.y crescente). Assim, numa pilha de objetos, o de baixo
    //    cai primeiro e o de cima o acompanha no mesmo quadro sem ficar
    //    flutuando por um passo.
    // -----------------------------------------------------------------------
    std::vector<size_t> order;
    order.reserve(boxes.size());
    for (size_t k = 0; k < boxes.size(); ++k)
    {
        int idx = owner[k];
        if (idx >= 0 && g_Instances[idx].grabbable)
            order.push_back(k);
    }
    std::sort(order.begin(), order.end(),
              [&boxes](size_t a, size_t b) { return boxes[a].min.y < boxes[b].min.y; });

    for (size_t m = 0; m < order.size(); ++m)
    {
        size_t k   = order[m];
        int    idx = owner[k];

        Instance& inst = g_Instances[idx];
        if (idx == held_instance)
        {
            inst.vel_y = 0.0f;
            continue;
        }

        AABB& box = boxes[k];

        // Euler semi-implicito: v(t+dt) = v(t) - g*dt ; dy = v(t+dt)*dt.
        float v = inst.vel_y - PhysicsConfig::GRAVITY * dt;
        if (v < -PhysicsConfig::MAX_FALL_SPEED)
            v = -PhysicsConfig::MAX_FALL_SPEED;

        float dy = v * dt;

        // Superficie mais alta embaixo do objeto. Se a queda deste quadro
        // passaria dela, o objeto para exatamente em cima e zera a velocidade.
        float support = 0.0f;
        if (FindSupportHeight(box, boxes, (int)k,
                              PhysicsConfig::SUPPORT_TOL, &support)
            && box.min.y + dy <= support)
        {
            float snap_dy = support - box.min.y;
            // Evita que a cabeca do jogador em movimento empurre para cima um
            // objeto ja apoiado em outra superficie.
            if (snap_dy > 0.0f && std::fabs(support - boxes[player_k].max.y) < 1e-4f)
                snap_dy = 0.0f;

            dy = snap_dy;
            v  = 0.0f;
        }

        inst.pos.y += dy;
        inst.vel_y  = v;

        box.min.y += dy;
        box.max.y += dy;
    }
}
