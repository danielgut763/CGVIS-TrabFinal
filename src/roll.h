#ifndef ROLL_H
#define ROLL_H

// Rolar (M8). So rola o que rola na vida real:
//   - SHAPE_SPHERE (bola): rola em qualquer direcao, sem deslizar.
//   - SHAPE_PAWN DEITADO: rola em torno do proprio eixo, so na direcao
//     perpendicular a ele, e como a base e mais larga que a cabeca descreve
//     um arco (rolamento de cone) em vez de uma reta.
// Dado, cubo magico, tijolo, coelho e peao em pe NAO rolam: tombam (tip.h).
//
// Divisao de trabalho com physics.cpp:
//   1. physics.cpp faz a queda vertical e o teste de apoio/equilibrio;
//   2. RollStep ajusta inst.vel no plano (aceleracao para fora da borda,
//      atrito de rolamento, restricao de direcao do peao);
//   3. physics.cpp move no plano com ResolveAABBMovement;
//   4. RollAfterMove gira o objeto pela distancia REALMENTE percorrida
//      (rolar sem deslizar: angulo = distancia / raio) e trata a batida em
//      parede (quique com perda).

#include "rigid.h"

namespace RollConfig
{
    // Valores conferidos em test/t_a3.cpp (bola de raio 0.18 m e peao de
    // 0.56 m da cena, gravidade de 12 m/s^2, passo de 1/240 s).

    // Aceleracao horizontal de quem tem o centro alem da borda do apoio. Uma
    // esfera girando sobre a aresta tem aceleracao horizontal ~ 5/7 g (o/r),
    // que vai de zero ate ~8.6 m/s^2. Como a caixa de colisao (AABB) so perde
    // o apoio depois de o centro andar um raio inteiro alem da borda, a bola
    // fica "pairando" alem da quina durante esse trecho; com 5 m/s^2 ele dura
    // ~0.25 s e a bola sai a ~1.3 m/s (a real sairia a ~0.7 m/s, mas
    // escorregando devagar da quina, o que no AABB parece flutuar).
    const float EDGE_ACCEL        = 5.0f;  // m/s^2

    // Atrito de rolamento (desaceleracao constante). O real, num piso duro, e
    // bem menor (~0.1 m/s^2), mas a sala tem 12 x 9 m: com 0.35 uma bola que
    // cai de um bloco de 1.2 m anda ~2 m e para em ~3.7 s, sem ficar
    // quicando de parede em parede e mexendo nas pecas do puzzle.
    const float SPHERE_FRICTION   = 0.35f; // m/s^2

    // O peao nao e um cilindro: rola apoiado so na borda da base e na cabeca,
    // descrevendo um arco de ~0.64 m de raio. Com 1.2 m/s^2 um peao a
    // 1.5 m/s anda ~0.9 m e gira ~85 graus no arco antes de parar (~1.2 s):
    // da para ver a curva sem ele dar voltas pela sala.
    const float PAWN_FRICTION     = 1.20f; // m/s^2

    // Fracao da velocidade normal a parede que volta na batida (e = 0.35).
    // Bola de madeira/borracha dura em parede rigida fica entre 0.3 e 0.6; o
    // valor baixo evita ping-pong entre paredes proximas.
    const float WALL_RESTITUTION  = 0.35f;

    // Abaixo disto o rolamento para (REST, velocidades zeradas). Com o atrito
    // acima, a 3 cm/s faltariam menos de 2 mm ate parar sozinho. Fica acima
    // de PhysicsConfig::SLEEP_LINEAR (0.02) para o rolamento parar por aqui,
    // e nao pelo sono generico.
    const float STOP_SPEED        = 0.03f; // m/s

    // Perda no pouso, ALEM da troca entre deslizar e rolar que RollOnLand
    // calcula pelo momento angular (ver roll.cpp). O quique vertical nao e
    // simulado; esses 20% representam a energia que ele levaria.
    const float LAND_KEEP         = 0.80f;

    // Teto da velocidade de rolamento. A mao solta no maximo a 2 m/s e a
    // borda acelera pouco, entao so e atingido em quedas altas; a 4 m/s um
    // passo anda 1.7 cm, bem menos que a espessura de uma parede (0.2 m).
    const float MAX_SPEED         = 4.0f;  // m/s

    // Momento de inercia em torno do eixo de rolamento, em unidades de m r^2
    // (I = k m r^2), usado no pouso (RollOnLand): v = (v0 + k v_giro)/(1 + k).
    const float SPHERE_INERTIA    = 0.40f; // esfera macica: 2/5
    const float PAWN_INERTIA      = 0.50f; // cilindro macico: 1/2
}

// Verdadeiro se, na pose atual, o objeto rola: esfera, ou peao deitado.
bool RollsNow(const Instance& inst);

// Para um objeto que RollsNow e esta desequilibrado: true se ele sai
// rolando para o lado de "overhang". Falso para o peao deitado pendurado na
// direcao do proprio eixo (ai ele tomba de ponta, como caixa).
bool RollHandlesOverhang(const Instance& inst, const glm::vec3& overhang);

// Pouso de um objeto que rola: converte o que sobrou do movimento
// (inst.vel horizontal e body.ang_vel) em velocidade de rolamento valida
// (peao: so a componente perpendicular ao eixo) e passa a MOTION_ROLL, ou a
// MOTION_REST se a velocidade for desprezivel. Conservacao do momento angular
// em relacao ao ponto de contato: v = LAND_KEEP (v0 + k v_giro)/(1 + k), com
// v_giro = r (omega x up); uma bola que saiu rolando de uma borda pousa com
// LAND_KEEP da velocidade. Nao mexe em vel.y. Nao endireita um peao que
// chegou torto: isso e o SettleBegin que physics.cpp chama depois.
void RollOnLand(Instance& inst, PhysicsContext& ctx);

// Um passo de rolamento sobre o apoio: altera so inst.vel.x/z (e o estado).
// Desequilibrado: acelera para o lado de sup.overhang (MOTION_ROLL).
// Equilibrado: atrito de rolamento; abaixo de STOP_SPEED passa a MOTION_REST
// com vel.x/z e body.ang_vel zerados.
void RollStep(Instance& inst, PhysicsContext& ctx, const SupportInfo& sup, float dt);

// Depois do movimento horizontal "applied" (com hit[eixo] de
// ResolveAABBMovement): gira sem deslizar, aplica o arco do peao (yaw) e o
// quique. Atualiza (*ctx.boxes)[ctx.self]. Em MOTION_ROLL deixa body.ang_vel
// com a velocidade angular de rolamento (up x v)/r (no peao, so a parte em
// torno do proprio eixo), para o objeto seguir girando no ar se sair da borda.
// Em MOTION_FALL (rolante no ar batendo numa parede) so faz o quique.
void RollAfterMove(Instance& inst, PhysicsContext& ctx,
                   const glm::vec3& applied, const bool hit[3], float dt);

#endif // ROLL_H
