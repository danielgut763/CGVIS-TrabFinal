#include "player.h"
#include "matrices.h"

#include <cmath>

void PlayerInit(Player& p, const glm::vec3& feet_pos)
{
    p.pos               = feet_pos;
    p.vel               = glm::vec3(0.0f);
    p.half_width        = PlayerConfig::HALF_WIDTH;
    p.height            = PlayerConfig::HEIGHT;
    p.eye_height        = PlayerConfig::EYE_HEIGHT;
    p.on_ground         = false;
    p.support_box_index = -1;
}

AABB PlayerAABB(const Player& p)
{
    AABB box;
    box.min = glm::vec3(p.pos.x - p.half_width,
                        p.pos.y,
                        p.pos.z - p.half_width);
    box.max = glm::vec3(p.pos.x + p.half_width,
                        p.pos.y + p.height,
                        p.pos.z + p.half_width);
    return box;
}

glm::vec3 PlayerEyePosition(const Player& p)
{
    return glm::vec3(p.pos.x, p.pos.y + p.eye_height, p.pos.z);
}

void UpdatePlayer(Player& p,
                  const PlayerInput& input,
                  float camera_yaw,
                  const std::vector<AABB>& solid_boxes,
                  float dt)
{
    if (dt <= 0.0f)
        return;

    // -----------------------------------------------------------------------
    // 1. Direcao de caminhada no plano horizontal XZ a partir do yaw da camera.
    //    Usamos crossproduct e norm de matrices.h para construir a base
    //    horizontal e normalizar o vetor de entrada (evitando que andar na
    //    diagonal seja sqrt(2) vezes mais rapido).
    // -----------------------------------------------------------------------
    glm::vec4 forward_xz(std::cos(camera_yaw), 0.0f, std::sin(camera_yaw), 0.0f);
    glm::vec4 up_world(0.0f, 1.0f, 0.0f, 0.0f);
    glm::vec4 right_xz = crossproduct(forward_xz, up_world);

    glm::vec4 move_dir(0.0f, 0.0f, 0.0f, 0.0f);
    if (input.forward)  move_dir += forward_xz;
    if (input.backward) move_dir -= forward_xz;
    if (input.right)    move_dir += right_xz;
    if (input.left)     move_dir -= right_xz;

    float len = norm(move_dir);
    if (len > 1e-5f)
        move_dir = move_dir / len;

    p.vel.x = move_dir.x * PlayerConfig::MOVE_SPEED;
    p.vel.z = move_dir.z * PlayerConfig::MOVE_SPEED;

    // -----------------------------------------------------------------------
    // 2. Pulo: permitido apenas quando o jogador esta apoiado no piso ou em
    //    cima de algum objeto solido.
    // -----------------------------------------------------------------------
    if (input.jump && p.on_ground)
    {
        p.vel.y     = PlayerConfig::JUMP_SPEED;
        p.on_ground = false;
    }

    // -----------------------------------------------------------------------
    // 3. Gravidade com integracao de Euler semi-implicito (simpletico):
    //    primeiro atualizamos a velocidade v(t + dt) = v(t) + a * dt, e em
    //    seguida calculamos o deslocamento desejado dx = v(t + dt) * dt usando
    //    a velocidade ja atualizada.
    // -----------------------------------------------------------------------
    p.vel.y -= PlayerConfig::GRAVITY * dt;
    if (p.vel.y < -PlayerConfig::MAX_FALL_SPEED)
        p.vel.y = -PlayerConfig::MAX_FALL_SPEED;

    glm::vec3 desired_delta = p.vel * dt;

    // -----------------------------------------------------------------------
    // 4. Colisao jogador x cena resolvida eixo a eixo em collisions.cpp.
    // -----------------------------------------------------------------------
    bool hit_axis[3] = { false, false, false };
    glm::vec3 applied_delta = ResolveAABBMovement(PlayerAABB(p),
                                                  desired_delta,
                                                  solid_boxes,
                                                  -1,
                                                  hit_axis);

    p.pos += applied_delta;

    // Se houve bloqueio no eixo X ou Z, zeramos a componente de velocidade.
    if (hit_axis[0]) p.vel.x = 0.0f;
    if (hit_axis[2]) p.vel.z = 0.0f;

    // Se houve bloqueio no eixo Y:
    //   - caindo (desired_delta.y < 0): o jogador pousou no piso ou no topo
    //     de um objeto;
    //   - subindo (desired_delta.y > 0): bateu a cabeca no teto ou na parte de
    //     baixo de um obstaculo.
    if (hit_axis[1])
    {
        if (desired_delta.y < 0.0f)
            p.on_ground = true;
        p.vel.y = 0.0f;
    }
    else
    {
        p.on_ground = false;
    }

    // -----------------------------------------------------------------------
    // 5. Teste Ponto em AABB (FindBoxContainingPoint) logo abaixo dos pes para
    //    identificar sobre qual caixa solida (piso ou objeto empilhado) o
    //    jogador esta apoiado.
    // -----------------------------------------------------------------------
    if (p.on_ground)
    {
        glm::vec3 probe(p.pos.x, p.pos.y - 0.02f, p.pos.z);
        p.support_box_index = FindBoxContainingPoint(probe, solid_boxes, -1);
    }
    else
    {
        p.support_box_index = -1;
    }
}
