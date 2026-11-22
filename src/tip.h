#ifndef TIP_H
#define TIP_H

// Tombar (M8). Objetos que NAO rolam (dado, cubo magico, tijolo, coelho, e o
// peao em pe) perdem o equilibrio girando em torno da ARESTA do apoio, como
// um cubo empurrado para fora da borda de uma mesa, em vez de escorregar.
//
// Ciclo (estados em scene.h, MotionState):
//   REST --(centro de massa fora da area de apoio)--> TIP
//   TIP  --(passou do ponto de soltar)--------------> FALL (com vel e ang_vel)
//   FALL --(pousou torto)---------------------------> SETTLE
//   SETTLE --(chegou na pose estavel)---------------> REST (ou ROLL, roll.h)
//
// Quem decide QUANDO chamar cada funcao e physics.cpp. Toda funcao que muda
// posicao ou orientacao atualiza (*ctx.boxes)[ctx.self] com
// InstanceCollisionBox, para os proximos objetos do mesmo passo enxergarem a
// caixa nova.

#include "rigid.h"

namespace TipConfig
{
    // Separacao da face (tip.cpp, TipStep): o objeto larga a aresta quando
    // omega^2 h >= g cos(theta), o que da 51-57 graus para caixas e para o
    // peao (medido em test/t_a2.cpp: 51,6 graus para o cubo de 0,36 m com
    // 3,6 cm para fora; 54 graus para o peao). RELEASE_ANGLE e so o teto de
    // seguranca para quando a separacao nao dispara (objeto de poucos cm cuja
    // base nao desce abaixo de SUPPORT_TOL). Fica abaixo de 90 graus para a
    // velocidade de saida ainda apontar para baixo e para fora.
    const float RELEASE_ANGLE   = 1.20f; // rad (~69 graus): teto do giro sobre a aresta

    // Seguranca contra um tombo que nao progride. O tombo livre mais lento e
    // o de um objeto grande que comeca quase equilibrado (overhang ~
    // BALANCE_MARGIN): pendulo invertido, t ~ ln(theta_f/theta_0) / k com
    // k = sqrt(g L / (I_pivo/m)). Um cubo de 2 m saindo com 6 mm para fora
    // leva 2,2 s ate soltar (t_a2 e2), e como t escala com sqrt(tamanho), um
    // de 3 m (o maior que cabe na sala) leva ~2,7 s: 4 s nunca corta um
    // tombo de verdade.
    // Escorado (encostou em parede/jogador) o contador fica zerado.
    const float MAX_TIP_TIME    = 4.00f; // s

    // Assentar: SETTLE_TIME e a duracao para meia-aresta SETTLE_REF_HALF
    // (o cubo de 0,36 m do caso de teste); objetos maiores levam mais, na
    // proporcao sqrt(tamanho), como um pendulo (o dado de 1,5 m leva 0,31 s),
    // limitado a [SETTLE_SCALE_MIN, SETTLE_SCALE_MAX] x SETTLE_TIME. 0,15 s
    // e da ordem do tempo de queda de um cubo de 0,36 m deitando sobre a
    // face, sqrt(I_aresta / (m g L)) ~ 0,17 s, entao o assentamento nao
    // parece nem um estalo nem camera lenta.
    const float SETTLE_TIME      = 0.15f; // s
    const float SETTLE_REF_HALF  = 0.18f; // m
    const float SETTLE_SCALE_MIN = 0.6f;
    const float SETTLE_SCALE_MAX = 2.5f;

    // Abaixo disto (~1,1 grau) a pose ja conta como estavel e so passa por
    // CanonicalizePose, sem animacao. Corrige o erro numerico de quem pousou
    // reto (soltar da mao, queda sem giro) sem assentamento visivel; o salto
    // da correcao e menor que 1 cm mesmo no dado de 1,5 m (0,75 * 0,02).
    const float STABLE_TOL       = 0.02f; // rad

    // Rotacao no ar sem amortecimento: o arrasto do ar num objeto desses em
    // menos de 1 s de queda e desprezivel, e w constante e o que a fisica
    // diz (a gravidade nao faz torque em relacao ao centro de massa).
    const float AIR_SPIN_DAMP    = 0.0f;  // 1/s

    // TipBegin testa se cabe um giro deste tamanho sobre a aresta antes de
    // comecar. Se nem ~3 graus cabem (face encostada ou a menos de ~5% do
    // tamanho de uma parede, outro objeto em cima, jogador colado), o objeto
    // nao tomba e physics.cpp usa o deslizamento do M7, que para no
    // obstaculo. Com folga maior ele tomba e fica escorado (ver TipStep).
    const float PROBE_ANGLE      = 0.05f; // rad
}

// Comeca um tombo: escolhe a aresta de apoio do lado de sup.overhang (borda
// da area de apoio, alinhada aos eixos do mundo porque o apoio e uma AABB;
// eixo dominante do overhang), pivo = ponto da aresta na altura do apoio na
// coordenada do centro, axis = up x (direcao para fora). Guarda pivot e axis
// em inst.body, zera inst.vel, ang_vel, angle e timer e passa a MOTION_TIP.
// Devolve false se nao da para tombar (overhang nulo, centro nao esta para
// fora da aresta, ou nao cabe um giro de PROBE_ANGLE); nesse caso
// physics.cpp cai no deslizamento antigo.
bool TipBegin(Instance& inst, PhysicsContext& ctx, const SupportInfo& sup);

// Avanca o tombo um passo: corpo rigido girando em torno da aresta fixa,
// I_pivo * alpha = m g (braco horizontal), I_pivo = I_cm + m d^2 (Steiner),
// Euler semi-implicito e RotateAboutLine. body.ang_vel = omega * axis o tempo
// todo. Solta (MOTION_FALL com inst.vel = omega x (centro - pivo) e
// body.ang_vel = omega * axis) quando a face de baixo se separa da aresta
// (omega^2 h >= g cos(theta)), ou em RELEASE_ANGLE, ou em MAX_TIP_TIME.
// Se o giro do passo faria a caixa entrar em algo que nao e o apoio
// (parede, outro objeto, jogador), gira so ate encostar, zera omega e fica
// ESCORADO em MOTION_TIP (pode ficar assim indefinidamente; volta a tombar
// sozinho se o obstaculo sair). Se nao ha mais caixa sob a aresta (o jogador
// pegou o apoio), solta para MOTION_FALL: physics.cpp nao precisa testar o
// apoio durante o TIP (a AABB inclinada desce abaixo do topo do apoio e
// FindSupportHeight deixa de ve-lo).
void TipStep(Instance& inst, PhysicsContext& ctx, float dt);

// Rotacao livre no ar: gira em torno do centro com body.ang_vel*dt. Se a
// caixa nova entraria em algo em que a antiga ainda nao estava entrando,
// nao gira neste passo (ang_vel e mantido e o giro continua quando a caixa
// sair do obstaculo). Sobreposicao que ja existia e tolerada: logo depois de
// largar a aresta a AABB inclinada ainda cobre o canto do apoio, sem que o
// corpo de verdade o atravesse.
void AirSpinStep(Instance& inst, PhysicsContext& ctx, float dt);

// Pouso de objeto torto: se NearestStablePose diz que nao esta estavel, passa
// a MOTION_SETTLE (guarda eixo e angulo que faltam em body.axis/body.angle) e
// devolve true. Se ja esta estavel, CanonicalizePose, reapoia a base e
// devolve false. Nunca mexe em inst.vel.
bool SettleBegin(Instance& inst, PhysicsContext& ctx);

// Assentar: gira em torno do centro em direcao a pose estavel (SETTLE_TIME
// escalado pelo tamanho, com aceleracao constante) e, a cada passo, reapoia a
// base na altura do apoio (desce ate encostar, sem afundar). Nunca mexe em
// inst.vel. No fim: CanonicalizePose, zera ang_vel e passa a MOTION_ROLL se
// RollsNow e a velocidade horizontal > RollConfig::STOP_SPEED, senao
// MOTION_REST. Casos especiais: se sob a pegada nova nao ha mais apoio, passa
// a MOTION_FALL girando; se o giro entraria numa parede/objeto, para
// escorado em MOTION_REST na pose em que esta.
void SettleStep(Instance& inst, PhysicsContext& ctx, float dt);

#endif // TIP_H
