#ifndef PLAYER_H
#define PLAYER_H

// Movimentacao do jogador em primeira pessoa: caminhada no plano XZ (WASD),
// gravidade integrada por Euler semi-implicito, pulo (Espaco), colisao AABB x
// AABB resolvida por eixo (via collisions.cpp) e suporte sobre o piso ou em
// cima de objetos da cena.

#include <vector>
#include <glm/vec3.hpp>
#include "collisions.h"

namespace PlayerConfig
{
    const float HALF_WIDTH     = 0.25f; // largura total = 0.50 m em X e Z
    const float HEIGHT         = 1.60f; // altura total do corpo em metros
    const float EYE_HEIGHT     = 1.45f; // altura dos olhos a partir dos pes
    const float MOVE_SPEED     = 3.2f;  // velocidade horizontal em m/s
    const float GRAVITY        = 12.0f; // aceleracao da gravidade em m/s^2
    const float JUMP_SPEED     = 4.2f;  // velocidade vertical inicial do pulo (m/s)
    const float MAX_FALL_SPEED = 20.0f; // velocidade terminal de queda (m/s)
}

struct PlayerInput
{
    bool forward;  // tecla W
    bool backward; // tecla S
    bool left;     // tecla A
    bool right;    // tecla D
    bool jump;     // tecla Espaco
};

struct Player
{
    glm::vec3 pos;               // posicao do CENTRO DOS PES em mundo (x, y_min, z)
    glm::vec3 vel;               // velocidade em m/s
    float     half_width;        // meia-largura da AABB em X e Z
    float     height;            // altura da AABB em Y
    float     eye_height;        // altura da camera acima de pos.y
    bool      on_ground;         // verdadeiro quando apoiado no piso ou sobre um objeto
    int       support_box_index; // indice na lista de AABBs do objeto sob os pes (-1 se no ar)
};

// Inicializa o jogador com os pes em "feet_pos".
void PlayerInit(Player& p, const glm::vec3& feet_pos);

// Devolve a AABB do jogador em coordenadas de mundo.
AABB PlayerAABB(const Player& p);

// Posicao dos olhos do jogador em mundo (onde fica a camera em 1a pessoa).
glm::vec3 PlayerEyePosition(const Player& p);

// Avanca a fisica e a movimentacao do jogador por um passo de tempo "dt"
// (em segundos), resolvendo colisoes contra "solid_boxes".
void UpdatePlayer(Player& p,
                  const PlayerInput& input,
                  float camera_yaw,
                  const std::vector<AABB>& solid_boxes,
                  float dt);

#endif // PLAYER_H
