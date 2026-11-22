#include "roll.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "collisions.h"
#include "scene.h"

// ---------------------------------------------------------------------------
// Rolamento da bola e do peao deitado (M8). Ver roll.h para a divisao de
// trabalho com physics.cpp.
//
// Ideia central: ROLAR SEM DESLIZAR. O ponto do corpo que encosta no apoio
// esta parado em relacao a ele, entao a velocidade do centro e a velocidade
// angular ficam amarradas:
//
//     v_centro = omega x (c - P) = omega x (r * up)      (P = ponto de contato)
//  => omega    = (up x v) / r
//
// Em vez de integrar omega, giramos o objeto pela distancia que ele REALMENTE
// andou neste passo (depois da colisao): eixo up x d/|d| e angulo |d|/r. Se
// uma parede truncou o movimento, o giro e truncado junto, e a textura nunca
// "patina" no piso.
// ---------------------------------------------------------------------------

namespace
{
    const glm::vec3 UP(0.0f, 1.0f, 0.0f);

    // Vetores escritos a mao (o enunciado veta glm::dot/cross/length).
    float Dot3(const glm::vec3& a, const glm::vec3& b)
    {
        return a.x*b.x + a.y*b.y + a.z*b.z;
    }

    glm::vec3 Cross3(const glm::vec3& a, const glm::vec3& b)
    {
        return glm::vec3(a.y*b.z - a.z*b.y,
                         a.z*b.x - a.x*b.z,
                         a.x*b.y - a.y*b.x);
    }

    float HorizLen(const glm::vec3& v)
    {
        return std::sqrt(v.x*v.x + v.z*v.z);
    }

    // -----------------------------------------------------------------------
    // Perfil do peao (data/pawn.obj, coordenadas do modelo; o eixo de
    // revolucao e o Y local, base plana em -Y). Medido nos vertices: raio
    // maximo de cada "anel" da malha em funcao de y.
    //   - borda da base: raio 0.335 em y = -0.4625 (e o raio XZ da bbox);
    //   - a reta que encosta na borda da base e na cabeca sem cortar o perfil
    //     (aresta do fecho convexo) toca a cabeca em raio 0.1655, y = 0.3468.
    //     O colarinho (raio 0.20 em y = 0.15) fica por dentro dessa reta, entao
    //     o peao deitado de verdade encosta no piso so na base e na cabeca.
    // -----------------------------------------------------------------------
    const float PAWN_BASE_R = 0.335f;
    const float PAWN_BASE_Y = -0.4625f;
    const float PAWN_HEAD_R = 0.1655f;
    const float PAWN_HEAD_Y = 0.3468f;

    // Raio de rolamento:
    //   esfera: metade do maior lado da bbox local vezes a escala (o mesmo
    //           raio que BodyWorldAABB usa para a caixa centro +- r);
    //   peao deitado: raio XZ da bbox vezes a escala. E o raio do cilindro de
    //           BodyWorldAABB, ou seja, a altura do centro acima do piso: a
    //           fisica apoia o peao deitado sobre esse cilindro.
    float RollRadius(const Instance& inst, const glm::vec3& lmin, const glm::vec3& lmax)
    {
        float sx = (lmax.x - lmin.x) * std::fabs(inst.scale.x);
        float sy = (lmax.y - lmin.y) * std::fabs(inst.scale.y);
        float sz = (lmax.z - lmin.z) * std::fabs(inst.scale.z);

        if (inst.shape == SHAPE_SPHERE)
            return 0.5f * std::max(sx, std::max(sy, sz));

        return 0.5f * std::max(sx, sz);
    }

    // -----------------------------------------------------------------------
    // Raio do arco do peao (rolamento de cone).
    //
    // Deitado, o peao encosta no piso ao longo da reta que toca a borda da
    // base (raio rb) e a cabeca (raio rh), separadas por l = yh - yb ao longo
    // do eixo. Prolongando essa reta ate o raio chegar a zero, achamos o
    // vertice V do cone equivalente, alem da cabeca. Como o raio cresce
    // linearmente com a distancia a V, por semelhanca de triangulos:
    //
    //     rb / Lb = rh / (Lb - l)   =>   Lb = l * rb / (rb - rh)
    //
    // com Lb a distancia (axial) da base ate V. Rolando sem deslizar, cada
    // ponto da reta de contato anda proporcionalmente a sua distancia a V
    // (a base, de raio maior, anda mais que a cabeca), entao o corpo inteiro
    // gira em torno da vertical que passa por V. O centro da bbox (yc) esta a
    //
    //     R = Lb - (yc - yb)
    //
    // de V, e e esse o raio do arco que o centro descreve:  d(yaw)/dt = v / R.
    // Com os numeros do perfil: l = 0.809, Lb = 1.600, R = 1.137 (modelo),
    // ou seja, o vertice fica 0.64 alem da ponta da cabeca (y = 0.5).
    // A escala do eixo (scale.y) multiplica as distancias axiais.
    // -----------------------------------------------------------------------
    float PawnArcRadius(const Instance& inst, const glm::vec3& lmin, const glm::vec3& lmax)
    {
        const float l  = PAWN_HEAD_Y - PAWN_BASE_Y;
        const float Lb = l * PAWN_BASE_R / (PAWN_BASE_R - PAWN_HEAD_R);
        const float yc = 0.5f * (lmin.y + lmax.y);
        return (Lb - (yc - PAWN_BASE_Y)) * std::fabs(inst.scale.y);
    }

    // Eixo do peao projetado no plano horizontal e normalizado. Falso se o
    // eixo esta (quase) na vertical, quando nao ha direcao de rolamento.
    bool PawnHorizontalAxis(const Instance& inst, glm::vec3* out)
    {
        glm::vec3 a = InstanceLocalAxisWorld(inst, 1);
        float n = HorizLen(a);
        if (n < 1e-4f)
            return false;
        *out = glm::vec3(a.x / n, 0.0f, a.z / n);
        return true;
    }

    // Componente horizontal de v perpendicular ao eixo horizontal "ah":
    //     v_perp = v - (v . ah) ah
    glm::vec3 ProjectPerp(const glm::vec3& v, const glm::vec3& ah)
    {
        glm::vec3 h(v.x, 0.0f, v.z);
        return h - ah * Dot3(h, ah);
    }

    // Peao deitado de fato rolando (para os casos em que RollsNow ja garantiu
    // o "deitado", so falta o eixo horizontal valido).
    bool IsLyingPawn(const Instance& inst, glm::vec3* ah)
    {
        return inst.shape == SHAPE_PAWN && PawnIsLying(inst) && PawnHorizontalAxis(inst, ah);
    }

    void ClampSpeed(glm::vec3& v)
    {
        float s = HorizLen(v);
        if (s > RollConfig::MAX_SPEED)
        {
            v.x *= RollConfig::MAX_SPEED / s;
            v.z *= RollConfig::MAX_SPEED / s;
        }
    }

    // "a" entra em "b" mais que EPS nos tres eixos. Encostar face com face
    // (objeto apoiado no piso, encostado na parede depois de uma batida) NAO
    // conta. A folga e bem menor que a de ResolveAABBMovement (1e-4) para o
    // arco do peao encostado numa parede nao "morder" a parede: so absorve o
    // erro de arredondamento da caixa (da ordem de 1e-7 m).
    bool Penetrates(const AABB& a, const AABB& b)
    {
        const float EPS = 1e-6f;
        for (int i = 0; i < 3; ++i)
            if (a.max[i] <= b.min[i] + EPS || a.min[i] >= b.max[i] - EPS)
                return false;
        return true;
    }

    // Verdadeiro se a caixa nova entra em alguma caixa da cena (fora a
    // propria) em que a caixa antiga nao entrava.
    bool EntersSomething(const AABB& novo, const AABB& antigo,
                         const std::vector<AABB>& boxes, int self)
    {
        for (size_t i = 0; i < boxes.size(); ++i)
        {
            if ((int)i == self)
                continue;
            if (Penetrates(novo, boxes[i]) && !Penetrates(antigo, boxes[i]))
                return true;
        }
        return false;
    }

    // Velocidade angular de rolamento correspondente a velocidade horizontal
    // v (ver o topo do arquivo): omega = (up x v) / r. Para o peao, so a
    // parte em torno do proprio eixo a, (omega . a) a: e ela que o RollOnLand
    // converte de volta em velocidade. O giro do arco (em torno da vertical)
    // fica de fora de proposito: no ar, girar em torno de um eixo fixo que
    // mistura o eixo do peao com a vertical faria o eixo do peao descrever um
    // cone em torno dele (precessao), e o peao chegaria torto ao piso.
    glm::vec3 RollingAngularVelocity(const Instance& inst, const PhysicsContext& ctx,
                                     const glm::vec3& v)
    {
        float r = RollRadius(inst, ctx.lmin, ctx.lmax);
        if (r < 1e-6f)
            return glm::vec3(0.0f);

        glm::vec3 h(v.x, 0.0f, v.z);
        glm::vec3 w = Cross3(UP, h) * (1.0f / r);

        if (inst.shape == SHAPE_PAWN)
        {
            glm::vec3 a = InstanceLocalAxisWorld(inst, 1);
            return a * Dot3(w, a);
        }
        return w;
    }

    void StopRolling(Instance& inst)
    {
        inst.vel.x = 0.0f;
        inst.vel.z = 0.0f;
        inst.body.ang_vel = glm::vec3(0.0f);
        if (inst.body.motion != MOTION_REST)
            inst.body.timer = 0.0f;
        inst.body.motion = MOTION_REST;
    }
}

bool RollsNow(const Instance& inst)
{
    if (inst.shape == SHAPE_SPHERE)
        return true;
    if (inst.shape == SHAPE_PAWN)
        return PawnIsLying(inst);
    return false;
}

// ---------------------------------------------------------------------------
// Peao deitado pendurado: decompomos o vetor "overhang" (do apoio ate o
// centro, horizontal) na direcao do eixo e na perpendicular:
//     o_par  = |o . ah|            (pendurado pela ponta: base ou cabeca)
//     o_perp = |o - (o . ah) ah|   (pendurado de lado)
// De lado, o peao rola para fora como um cilindro. Pela ponta, ele nao pode
// rolar nessa direcao (rolar e sempre perpendicular ao eixo): a ponta cai e
// ele gira sobre a aresta do apoio, que e o tombo de tip.h.
// ---------------------------------------------------------------------------
bool RollHandlesOverhang(const Instance& inst, const glm::vec3& overhang)
{
    if (!RollsNow(inst))
        return false;
    if (inst.shape == SHAPE_SPHERE)
        return true;

    glm::vec3 ah;
    if (!PawnHorizontalAxis(inst, &ah))
        return false;

    glm::vec3 o(overhang.x, 0.0f, overhang.z);
    float o_par  = std::fabs(Dot3(o, ah));
    float o_perp = HorizLen(ProjectPerp(o, ah));
    return o_perp >= o_par;
}

// ---------------------------------------------------------------------------
// Pouso. Durante o impacto as unicas forcas grandes (normal e atrito) agem no
// ponto de contato P, entao o momento angular em relacao a P se conserva.
// Com c - P = r*up e I = k*m*r^2 em torno do eixo de rolamento:
//
//   antes:  L = m r (up x v0) + I w0
//   depois: rolando sem deslizar, w = (up x v)/r, L = (m r + I/r)(up x v)
//
// Escrevendo a parte horizontal de w0 como uma "velocidade de superficie"
// v_giro = r (w0 x up) (a velocidade que o giro sozinho daria rolando; vale
// up x v_giro / r = w0 horizontal), sai
//
//     v = (v0 + k v_giro) / (1 + k)
//
// - bola que saiu rolando da borda: w0 = (up x v0)/r, v_giro = v0, v = v0
//   (continua com a mesma velocidade, como na vida real);
// - bola sem giro: v = v0/(1+k), 5/7 da velocidade (o atrito freia e faz
//   girar);
// - peao: so conta a componente perpendicular ao eixo (na direcao do eixo o
//   atrito do piso para o deslizamento) e so o giro em torno do proprio eixo
//   a: w0.a vezes r na direcao (a x up). Um peao que tombou de pe cai com
//   giro em torno da aresta (perpendicular ao eixo) e velocidade quase toda
//   ao longo do eixo, entao quase nao rola: e o que se ve com um peao real.
// Por fim LAND_KEEP desconta a energia que o quique (nao simulado) leva.
// ---------------------------------------------------------------------------
void RollOnLand(Instance& inst, PhysicsContext& ctx)
{
    if (!RollsNow(inst))
        return;

    float r = RollRadius(inst, ctx.lmin, ctx.lmax);
    glm::vec3 v0(inst.vel.x, 0.0f, inst.vel.z);
    glm::vec3 w0 = inst.body.ang_vel;
    glm::vec3 v_giro(0.0f);
    float k = RollConfig::SPHERE_INERTIA;

    glm::vec3 ah;
    const bool pawn = IsLyingPawn(inst, &ah);
    if (pawn)
    {
        k  = RollConfig::PAWN_INERTIA;
        v0 = ProjectPerp(v0, ah);
        // giro em torno do proprio eixo; (a x up) e horizontal e
        // perpendicular ao eixo, com |a x up| = 1 para o eixo deitado
        glm::vec3 a = InstanceLocalAxisWorld(inst, 1);
        float wa = Dot3(w0, a);
        v_giro = Cross3(ah, UP) * (wa * r);
    }
    else
    {
        glm::vec3 wh(w0.x, 0.0f, w0.z);
        v_giro = Cross3(wh, UP) * r;
    }

    glm::vec3 v = (v0 + v_giro * k) * (RollConfig::LAND_KEEP / (1.0f + k));
    if (pawn)
        v = ProjectPerp(v, ah);
    ClampSpeed(v);

    inst.body.ang_vel = glm::vec3(0.0f);
    inst.body.timer   = 0.0f;

    if (HorizLen(v) < RollConfig::STOP_SPEED)
    {
        StopRolling(inst);
        return;
    }

    inst.vel.x = v.x;
    inst.vel.z = v.z;
    inst.body.motion  = MOTION_ROLL;
    inst.body.ang_vel = RollingAngularVelocity(inst, ctx, v);
}

// ---------------------------------------------------------------------------
// Um passo sobre o apoio (so mexe em vel.x/z e no estado).
//
// Desequilibrado (centro alem da borda): a gravidade faz o corpo girar em
// torno da aresta de apoio. Para uma esfera de raio r com o centro a uma
// distancia o alem da aresta, o torque m g o sobre I_aresta = 7/5 m r^2 da
// aceleracao horizontal ~ 5/7 g (o/r). Usamos uma aceleracao constante
// EDGE_ACCEL na direcao do overhang (ver roll.h), que e a leitura visual
// "rola para fora da borda" sem depender do quanto ja passou.
//
// Equilibrado: atrito de rolamento, uma desaceleracao constante
// |v| <- |v| - f dt, ate STOP_SPEED; ai para (REST).
//
// Peao deitado: so rola perpendicular ao eixo, entao a velocidade e
// projetada:  v <- v - (v . ah) ah.
// ---------------------------------------------------------------------------
void RollStep(Instance& inst, PhysicsContext& ctx, const SupportInfo& sup, float dt)
{
    if (!RollsNow(inst))
        return;

    glm::vec3 ah;
    const bool pawn = IsLyingPawn(inst, &ah);

    glm::vec3 v(inst.vel.x, 0.0f, inst.vel.z);

    if (sup.unbalanced)
    {
        glm::vec3 o(sup.overhang.x, 0.0f, sup.overhang.z);
        if (pawn)
            o = ProjectPerp(o, ah);
        float ol = HorizLen(o);
        if (ol > 1e-6f)
            v += o * (RollConfig::EDGE_ACCEL * dt / ol);

        if (inst.body.motion != MOTION_ROLL)
            inst.body.timer = 0.0f;
        inst.body.motion = MOTION_ROLL;
    }
    else
    {
        if (pawn)
            v = ProjectPerp(v, ah);

        float s = HorizLen(v);
        float f = pawn ? RollConfig::PAWN_FRICTION : RollConfig::SPHERE_FRICTION;
        float s_new = s - f * dt;

        if (s_new <= RollConfig::STOP_SPEED)
        {
            StopRolling(inst);
            return;
        }
        v *= s_new / s;
    }

    if (pawn)
        v = ProjectPerp(v, ah);
    ClampSpeed(v);

    inst.vel.x = v.x;
    inst.vel.z = v.z;
}

// ---------------------------------------------------------------------------
// Depois do movimento horizontal.
//
// 1) Quique: em cada eixo bloqueado, v_eixo <- -WALL_RESTITUTION * v_eixo
//    (a parede so devolve parte da velocidade normal; a tangente segue).
//
// 2) Giro sem deslizar pela distancia d (horizontal) realmente andada:
//        eixo = up x d / |d|,  angulo = |d| / r
//    Conferencia do sinal: indo para +X, up x X = -Z; girar em torno de -Z
//    leva o topo da bola (0, r, 0) para +X, e o topo de uma roda anda mesmo
//    para a frente (a 2v) enquanto o ponto de baixo fica parado.
//    No peao o eixo de giro e o proprio eixo a do peao (com o sinal de
//    (up x d) . a) e o angulo usa so a parte de d perpendicular a ele:
//    |(up x d) . a| / r. Assim o eixo nunca "bambeia" se nao estiver
//    perfeitamente horizontal.
//
// 3) Arco do peao (ver PawnArcRadius): o centro gira em torno da vertical
//    por V, que fica a uma distancia R do centro na direcao do eixo (lado da
//    cabeca). Velocidade do centro em torno de V:
//        v = w_yaw up x (c - V) = -w_yaw R (up x ah)
//    entao  d(yaw) = -(d . (up x ah)) / R.
//    O centro ja andou d em linha reta; aqui so a orientacao gira de d(yaw)
//    em torno do proprio centro (yaw += d(yaw), Ry(yaw + dy) = Ry(dy)Ry(yaw)),
//    e a velocidade gira junto. No passo seguinte o centro anda na direcao
//    nova: o caminho vira o arco.
//
// Se o giro ou o arco fariam a caixa entrar em algo (o peao girando no plano
// muda a sua AABB), aplica so a fracao que cabe (bissecao, 4 tentativas).
// ---------------------------------------------------------------------------
void RollAfterMove(Instance& inst, PhysicsContext& ctx,
                   const glm::vec3& applied, const bool hit[3], float dt)
{
    std::vector<AABB>& boxes = *ctx.boxes;

    // 1) quique
    const glm::vec3 v_before = inst.vel;
    if (hit[0]) inst.vel.x = -RollConfig::WALL_RESTITUTION * v_before.x;
    if (hit[2]) inst.vel.z = -RollConfig::WALL_RESTITUTION * v_before.z;

    // No ar (bola que saiu da borda e bateu na parede) so ha o quique: quem
    // gira o objeto e AirSpinStep com body.ang_vel. A restricao de direcao do
    // peao NAO vale no ar: um peao que tombou de uma caixa e ja passou de
    // |u.y| < 0.5 ainda esta voando com a velocidade do tombo, quase toda ao
    // longo do proprio eixo; projeta-la aqui o faria cair reto, rente a caixa.
    if (inst.body.motion != MOTION_ROLL || !RollsNow(inst))
    {
        boxes[ctx.self] = InstanceCollisionBox(inst, ctx.instance, ctx.lmin, ctx.lmax);
        return;
    }

    glm::vec3 ah;
    const bool pawn = IsLyingPawn(inst, &ah);
    if (pawn)
    {
        glm::vec3 vp = ProjectPerp(inst.vel, ah);
        inst.vel.x = vp.x;
        inst.vel.z = vp.z;
    }

    const glm::vec3 d(applied.x, 0.0f, applied.z);
    const float dist = HorizLen(d);
    const float r    = RollRadius(inst, ctx.lmin, ctx.lmax);

    AABB ref = InstanceCollisionBox(inst, ctx.instance, ctx.lmin, ctx.lmax);

    // 2) giro sem deslizar
    if (dist > 1e-7f && r > 1e-6f)
    {
        glm::vec3 k = Cross3(UP, d) * (1.0f / dist);   // up x d/|d|
        float ang = dist / r;

        if (pawn)
        {
            glm::vec3 a = InstanceLocalAxisWorld(inst, 1);
            float s = Dot3(k, a);                       // |s| = |d_perp|/|d|
            k   = (s >= 0.0f) ? a : -a;
            ang = std::fabs(s) * dist / r;
        }

        if (ang > 0.0f)
        {
            const glm::vec3 pos0  = inst.pos;
            const glm::mat4 tilt0 = inst.tilt;
            RotateAboutCenter(inst, ctx.lmin, ctx.lmax, k, ang);

            AABB nb = InstanceCollisionBox(inst, ctx.instance, ctx.lmin, ctx.lmax);
            if (EntersSomething(nb, ref, boxes, ctx.self))
            {
                inst.pos  = pos0;
                inst.tilt = tilt0;
            }
            else
                ref = nb;
        }
    }

    // 3) arco do peao
    if (pawn && dist > 1e-7f)
    {
        float R = PawnArcRadius(inst, ctx.lmin, ctx.lmax);
        float dyaw = (R > 1e-6f) ? -Dot3(d, Cross3(UP, ah)) / R : 0.0f;

        if (dyaw != 0.0f)
        {
            const glm::vec3 c     = InstanceBodyCenter(inst, ctx.lmin, ctx.lmax);
            const glm::vec3 pos0  = inst.pos;
            const float     yaw0  = inst.yaw;
            float           tried = dyaw;
            float           done  = 0.0f;

            for (int attempt = 0; attempt < 4; ++attempt)
            {
                inst.yaw = yaw0 + tried;
                SetInstanceBodyCenter(inst, ctx.lmin, ctx.lmax, c);
                AABB nb = InstanceCollisionBox(inst, ctx.instance, ctx.lmin, ctx.lmax);
                if (!EntersSomething(nb, ref, boxes, ctx.self))
                {
                    done = tried;
                    break;
                }
                tried *= 0.5f;
            }

            if (done == 0.0f)
            {
                inst.yaw = yaw0;
                inst.pos = pos0;
            }
            else
            {
                // mantem o yaw em (-pi, pi] para nao crescer sem limite
                const float PI = 3.14159265358979f;
                if (inst.yaw >  PI) inst.yaw -= 2.0f * PI;
                if (inst.yaw < -PI) inst.yaw += 2.0f * PI;
                SetInstanceBodyCenter(inst, ctx.lmin, ctx.lmax, c);

                // a velocidade gira com o corpo: Ry(dy) * (vx, 0, vz)
                float cs = std::cos(done), sn = std::sin(done);
                float vx = inst.vel.x, vz = inst.vel.z;
                inst.vel.x =  cs * vx + sn * vz;
                inst.vel.z = -sn * vx + cs * vz;
            }
        }

        // e continua perpendicular ao eixo (agora girado)
        glm::vec3 ah2;
        if (PawnHorizontalAxis(inst, &ah2))
        {
            glm::vec3 vp = ProjectPerp(inst.vel, ah2);
            inst.vel.x = vp.x;
            inst.vel.z = vp.z;
        }
    }

    // Velocidade angular de rolamento em mundo, para a bola (ou o peao) que
    // sair da borda continuar girando no ar (AirSpinStep) e RollOnLand
    // reaproveitar no pouso.
    if (dt > 0.0f)
        inst.body.ang_vel = RollingAngularVelocity(inst, ctx, inst.vel);

    boxes[ctx.self] = InstanceCollisionBox(inst, ctx.instance, ctx.lmin, ctx.lmax);
}
