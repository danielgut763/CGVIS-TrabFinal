#include "physics.h"

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

        boxes.push_back(AABBTransform(lmin, lmax, InstanceModelMatrix(inst)));
        owner.push_back((int)i);
    }

    boxes.push_back(player_box);
    owner.push_back(-1);

    // -----------------------------------------------------------------------
    // 2) Queda de cada objeto pegavel.
    // -----------------------------------------------------------------------
    for (size_t k = 0; k < boxes.size(); ++k)
    {
        int idx = owner[k];
        if (idx < 0)
            continue;

        Instance& inst = g_Instances[idx];
        if (!inst.grabbable)
            continue;

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
            dy = support - box.min.y;   // pode ser levemente positivo: desfaz
            v  = 0.0f;                  // uma penetracao de milimetros
        }

        inst.pos.y += dy;
        inst.vel_y  = v;

        box.min.y += dy;
        box.max.y += dy;
    }
}
