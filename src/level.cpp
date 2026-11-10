#include "level.h"

#include <algorithm>

#include "grab.h"   // GetMeshLocalBBox

// AABB em mundo de uma instancia, a partir da bbox local da malha.
static bool InstanceAABB(const Instance& inst, AABB* out)
{
    glm::vec3 lmin, lmax;
    if (!GetMeshLocalBBox(inst.mesh, &lmin, &lmax))
        return false;

    *out = AABBTransform(lmin, lmax, InstanceModelMatrix(inst));
    return true;
}

void LevelInit(LevelState& L)
{
    L.initial = g_Instances;

    L.door_closed_pos = glm::vec3(0.0f);
    if (g_DoorInstance >= 0 && g_DoorInstance < (int)g_Instances.size())
        L.door_closed_pos = g_Instances[g_DoorInstance].pos;

    L.door_offset    = 0.0f;
    L.button_press   = 0.0f;
    L.button_pressed = false;
    L.button_active  = false;
    L.won            = false;
    L.elapsed        = 0.0f;
    L.win_time       = 0.0f;
}

void LevelRestart(LevelState& L)
{
    std::vector<Instance> initial = L.initial;
    g_Instances = initial;
    LevelInit(L);
}

bool LevelPlayerAtExit(const AABB& player_box)
{
    // Pes no nivel da plataforma e o centro ja passou da parede, dentro do
    // corredor.
    float cx = 0.5f * (player_box.min.x + player_box.max.x);

    return player_box.min.y >= Room::EXIT_Y0 - 0.2f
        && cx > Room::HALF_X + Room::THICKNESS + 0.3f;
}

void LevelUpdate(LevelState& L, int held_instance, const AABB& player_box, float dt)
{
    if (dt <= 0.0f)
        return;

    if (!L.won)
        L.elapsed += dt;

    // -----------------------------------------------------------------------
    // 1) Objetos que cairam para fora da sala (pela saida, por exemplo) voltam
    //    para o lugar de origem, para o puzzle nunca ficar sem solucao.
    // -----------------------------------------------------------------------
    for (size_t i = 0; i < g_Instances.size() && i < L.initial.size(); ++i)
    {
        Instance& inst = g_Instances[i];
        if (!inst.grabbable || (int)i == held_instance)
            continue;

        if (inst.pos.y < LevelConfig::KILL_Y)
            inst = L.initial[i];
    }

    // -----------------------------------------------------------------------
    // 2) Botao: sobreposicao entre a AABB de cada objeto e um volume fino
    //    logo acima da placa.
    // -----------------------------------------------------------------------
    AABB zone;
    zone.min = glm::vec3(Puzzle::BUTTON_X - 0.5f*Puzzle::BUTTON_SIZE,
                         -0.02f,
                         Puzzle::BUTTON_Z - 0.5f*Puzzle::BUTTON_SIZE);
    zone.max = glm::vec3(Puzzle::BUTTON_X + 0.5f*Puzzle::BUTTON_SIZE,
                         Puzzle::BUTTON_UP_H + 0.03f,
                         Puzzle::BUTTON_Z + 0.5f*Puzzle::BUTTON_SIZE);

    L.button_pressed = false;
    L.button_active  = false;

    for (size_t i = 0; i < g_Instances.size(); ++i)
    {
        const Instance& inst = g_Instances[i];
        if (!inst.grabbable || (int)i == held_instance)
            continue;

        AABB box;
        if (!InstanceAABB(inst, &box) || !AABBOverlap(box, zone))
            continue;

        L.button_pressed = true;

        // "Peso" do objeto: a altura dele. Como a escala e uniforme, a altura
        // cresce junto com o volume, e so um objeto aumentado pela mecanica
        // passa do limite.
        if (box.max.y - box.min.y >= Puzzle::MIN_ACTIVATE_HEIGHT)
            L.button_active = true;
    }

    // O jogador afunda o botao, mas e leve demais para abrir a porta.
    if (AABBOverlap(player_box, zone))
        L.button_pressed = true;

    // Animacao da placa descendo e subindo, por dt.
    {
        float target = L.button_pressed ? 1.0f : 0.0f;
        float step   = 8.0f * dt;
        if (L.button_press < target) L.button_press = std::min(target, L.button_press + step);
        else                         L.button_press = std::max(target, L.button_press - step);
    }

    if (g_ButtonInstance >= 0 && g_ButtonInstance < (int)g_Instances.size())
    {
        Instance& b = g_Instances[g_ButtonInstance];
        float h = Puzzle::BUTTON_UP_H
                + (Puzzle::BUTTON_DOWN_H - Puzzle::BUTTON_UP_H) * L.button_press;

        b.scale.y = h;
        b.pos.y   = 0.5f * h;

        if (L.button_active)       b.mat.pattern = PATTERN_BUTTON_ACTIVE;
        else if (L.button_pressed) b.mat.pattern = PATTERN_BUTTON_LIGHT;
        else                       b.mat.pattern = PATTERN_BUTTON_IDLE;
    }

    // -----------------------------------------------------------------------
    // 3) Porta: desce enquanto o botao esta ativo e sobe quando ele e solto.
    //    Antes de subir, testa se a nova posicao encostaria no jogador ou em
    //    algum objeto: se sim, ela espera, para nunca prender nada dentro.
    // -----------------------------------------------------------------------
    if (g_DoorInstance >= 0 && g_DoorInstance < (int)g_Instances.size())
    {
        Instance& door = g_Instances[g_DoorInstance];

        float target = L.button_active ? Puzzle::DOOR_TRAVEL : 0.0f;
        float step   = Puzzle::DOOR_SPEED * dt;

        if (L.door_offset < target)
        {
            L.door_offset = std::min(target, L.door_offset + step);
        }
        else if (L.door_offset > target)
        {
            float next = std::max(target, L.door_offset - step);

            Instance probe = door;
            probe.pos      = L.door_closed_pos - glm::vec3(0.0f, next, 0.0f);

            AABB door_box;
            bool blocked = false;

            if (InstanceAABB(probe, &door_box))
            {
                if (AABBOverlap(door_box, player_box))
                    blocked = true;

                for (size_t i = 0; i < g_Instances.size() && !blocked; ++i)
                {
                    const Instance& o = g_Instances[i];
                    if (!o.grabbable)
                        continue;

                    AABB ob;
                    if (InstanceAABB(o, &ob) && AABBOverlap(door_box, ob))
                        blocked = true;
                }
            }

            if (!blocked)
                L.door_offset = next;
        }

        door.pos = L.door_closed_pos - glm::vec3(0.0f, L.door_offset, 0.0f);
    }

    // -----------------------------------------------------------------------
    // 4) Vitoria.
    // -----------------------------------------------------------------------
    if (!L.won && LevelPlayerAtExit(player_box))
    {
        L.won      = true;
        L.win_time = L.elapsed;
    }
}
