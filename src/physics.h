#ifndef PHYSICS_H
#define PHYSICS_H

// Gravidade dos objetos da fase (M6).
//
// Todo objeto pegavel que NAO esta na mao do jogador cai ate encontrar uma
// superficie de apoio: o piso, o topo de outro objeto, a plataforma da saida
// ou a cabeca do proprio jogador. E isto que faz a mecanica de perspectiva
// forcada funcionar como no jogo de referencia: o objeto e solto com o
// tamanho e a posicao calculados pela distancia e, se ficou no ar (encostado
// numa parede, por exemplo), despenca ate o chao mantendo a escala nova.
//
// A integracao e a mesma do jogador (player.cpp): Euler semi-implicito,
// primeiro a velocidade e depois a posicao com a velocidade ja atualizada.
// O teste de apoio fica em collisions.cpp (FindSupportHeight).

#include "collisions.h"

namespace PhysicsConfig
{
    const float GRAVITY        = 12.0f; // m/s^2, igual a PlayerConfig::GRAVITY
    const float MAX_FALL_SPEED = 20.0f; // velocidade terminal (m/s)
    const float SUPPORT_TOL    = 0.02f; // penetracao maxima aceita como "apoiado"
}

// Avanca a queda de todos os objetos pegaveis por um passo "dt".
//   held_instance: indice em g_Instances do objeto na mao (-1 se nenhum);
//                  ele nao cai e tem a velocidade zerada.
//   player_box:    AABB do jogador, tratada como obstaculo para que um objeto
//                  nunca caia por dentro dele.
void UpdateFallingObjects(int held_instance, const AABB& player_box, float dt);

#endif // PHYSICS_H
