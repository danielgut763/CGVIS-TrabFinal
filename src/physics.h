#ifndef PHYSICS_H
#define PHYSICS_H

// Fisica dos objetos da fase (M6/M7/M8).
//
// Todo objeto pegavel que NAO esta na mao do jogador:
//
//   1. CAI ate encontrar uma superficie de apoio: o piso, o topo de outro
//      objeto, a plataforma da saida ou a cabeca do proprio jogador. E isto
//      que faz a mecanica de perspectiva forcada funcionar como no jogo de
//      referencia: o objeto e solto com o tamanho e a posicao calculados pela
//      distancia e, se ficou no ar, despenca mantendo a escala nova.
//
//   2. So fica em EQUILIBRIO se a projecao do seu centro de massa cair dentro
//      da area de apoio (collisions.cpp, SupportFootprint). Sem este teste, um
//      objeto ficava em pe apoiado num canto de 1 cm.
//
//   3. (M8) Quando perde o equilibrio, faz o que o objeto real faria:
//      - dado, cubo magico, tijolo, coelho e peao em pe TOMBAM: giram sobre a
//        aresta do apoio ate soltar, caem girando e assentam numa face
//        (tip.h);
//      - bola e peao deitado ROLAM para fora da borda, rolam no piso,
//        quicam nas paredes e param pelo atrito de rolamento (roll.h).
//      No M7 isto era aproximado por um DESLIZAMENTO horizontal, porque a
//      cena so tinha rotacao em Y. Com ENABLE_TIPPING e ENABLE_ROLLING em
//      false o deslizamento volta a ser o comportamento (igual ao M7).
//
// A integracao e a mesma do jogador (player.cpp): Euler semi-implicito,
// primeiro a velocidade e depois a posicao com a velocidade ja atualizada.
// O deslocamento horizontal passa por ResolveAABBMovement, entao um objeto
// deslizando ou rolando para no primeiro obstaculo em vez de atravessa-lo.

#include "collisions.h"

namespace PhysicsConfig
{
    const float GRAVITY        = 12.0f; // m/s^2, igual a PlayerConfig::GRAVITY
    const float MAX_FALL_SPEED = 20.0f; // velocidade terminal (m/s)
    const float SUPPORT_TOL    = 0.02f; // penetracao maxima aceita como "apoiado"

    // Deslizamento de um objeto desequilibrado: acelera a partir do repouso
    // ate a velocidade maxima. Valores baixos de proposito: a leitura visual
    // e de um objeto que "escorrega" da borda, nao de um chute.
    const float SLIDE_ACCEL     = 4.0f;  // m/s^2
    const float SLIDE_MAX_SPEED = 1.6f;  // m/s

    // Folga de equilibrio: o centro pode ficar ate isto para fora da area de
    // apoio sem o objeto cair. Evita que um objeto exatamente na borda fique
    // oscilando entre apoiado e deslizando por erro de ponto flutuante.
    const float BALANCE_MARGIN  = 0.005f;

    // --- M8: tombar, rolar e passo fixo ---------------------------------

    // Liga o tombo (tip.h) e o rolamento (roll.h). Com os dois em false, o
    // comportamento e exatamente o do M7: objetos desequilibrados deslizam.
    // Os testes sem janela compilam com -DPHYSICS_ENABLE_TIPPING=0
    // -DPHYSICS_ENABLE_ROLLING=0 para comparar com o M7 sem editar o codigo
    // (tools/check.sh); no jogo valem os padroes abaixo (ligados).
#ifndef PHYSICS_ENABLE_TIPPING
#define PHYSICS_ENABLE_TIPPING 1
#endif
#ifndef PHYSICS_ENABLE_ROLLING
#define PHYSICS_ENABLE_ROLLING 1
#endif
    const bool  ENABLE_TIPPING = (PHYSICS_ENABLE_TIPPING != 0);
    const bool  ENABLE_ROLLING = (PHYSICS_ENABLE_ROLLING != 0);

    // Passo fixo da fisica dos objetos (main.cpp acumula o delta time e chama
    // UpdateFallingObjects em passos deste tamanho). Independente do FPS,
    // e pequeno o bastante para a queda mais rapida (MAX_FALL_SPEED) andar
    // menos que a espessura de uma parede por passo.
    const float FIXED_STEP     = 1.0f / 240.0f; // s
    const int   MAX_SUBSTEPS   = 24;            // teto por quadro (0,1 s)

    // Sono: apoiado, equilibrado e quase parado por SLEEP_TIME, o objeto
    // dorme (REST com velocidades zeradas) e para de acumular erro.
    const float SLEEP_LINEAR   = 0.02f;  // m/s
    const float SLEEP_ANGULAR  = 0.05f;  // rad/s
    const float SLEEP_TIME     = 0.30f;  // s

    // Despenetracao lateral (M8). Um objeto que gira (tombo, assentamento,
    // rolamento do peao) muda de AABB, e a caixa nova pode terminar ENTRANDO
    // pelo lado em outra caixa: no fim de um tombo, por exemplo, a parte de
    // tras da caixa girada ainda cobre a quina do apoio. Como o apoio so e
    // procurado abaixo da base, uma caixa nessa situacao desceria por dentro
    // da outra. Por isso, com o M8 ligado, toda caixa que penetra outra pelo
    // lado mais que DEPEN_MIN e empurrada para fora pelo eixo de menor
    // penetracao, a no maximo DEPEN_SPEED (para nao "teleportar").
    const float DEPEN_MIN      = 0.002f; // m
    const float DEPEN_SPEED    = 2.0f;   // m/s
}

// Maquina de estados por objeto, a cada passo (M8; estados em scene.h):
//   - na mao: body zerado (BodyState()), vel zerada, nada mais acontece;
//   - TIP:    TipStep (tip.h) e pronto; se o apoio sumiu (o jogador pegou o
//             objeto de baixo), vira FALL;
//   - SETTLE: SettleStep (tip.h) e pronto; ao terminar, um objeto que rola
//             e ainda tem velocidade passa a ROLL;
//   - REST/FALL/ROLL:
//       1. queda vertical e apoio (como no M7);
//       2. no ar: AirSpinStep; pousou neste passo (estava FALL ou descendo
//          e agora esta apoiado e parado em Y): RollOnLand se RollsNow, e
//          SettleBegin se a pose e torta;
//       3. apoiado e desequilibrado: rola (RollsNow && RollHandlesOverhang),
//          senao tomba (TipBegin), senao desliza (M7);
//       4. apoiado e equilibrado: RollStep se rolando, senao atrito;
//       5. movimento horizontal por ResolveAABBMovement; quem rola chama
//          RollAfterMove (gira pela distancia andada e quica), os demais
//          param no eixo que bateu;
//       6. sono: apoiado, equilibrado e quase parado por SLEEP_TIME zera as
//          velocidades.
// Avanca a fisica de todos os objetos pegaveis por um passo "dt".
//   held_instance: indice em g_Instances do objeto na mao (-1 se nenhum);
//                  ele nao cai e tem a velocidade zerada.
//   player_box:    AABB do jogador, tratada como obstaculo para que um objeto
//                  nunca caia por dentro dele.
void UpdateFallingObjects(int held_instance, const AABB& player_box, float dt);

#endif // PHYSICS_H
