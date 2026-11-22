#include "physics.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "grab.h"   // GetMeshLocalBBox
#include "rigid.h"  // PhysicsContext, SupportInfo
#include "roll.h"
#include "scene.h"
#include "tip.h"

// ---------------------------------------------------------------------------
// Fisica dos objetos pegaveis soltos (M6/M7/M8). A maquina de estados por
// objeto esta descrita em physics.h; aqui ficam:
//   - a lista de caixas compartilhada pelo passo inteiro;
//   - a queda vertical, o apoio e o teste de equilibrio (M6/M7);
//   - a decisao de QUANDO tombar, rolar, assentar ou dormir (M8). O COMO
//     tombar e o COMO rolar ficam em tip.cpp e roll.cpp.
// ---------------------------------------------------------------------------

namespace
{
    float Length3(const glm::vec3& v)
    {
        return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    }

    float HorizontalSpeed(const glm::vec3& v)
    {
        return std::sqrt(v.x*v.x + v.z*v.z);
    }

    float Clamp(float x, float lo, float hi)
    {
        return (x < lo) ? lo : (x > hi ? hi : x);
    }

    // Despenetracao lateral (ver DEPEN_* em physics.h). Para cada caixa que
    // a do objeto penetra, a penetracao em cada eixo e o comprimento da
    // sobreposicao dos intervalos:
    //     o_i = min(a.max_i, b.max_i) - max(a.min_i, b.min_i)
    // Contato de apoio (o_y pequeno, ate SUPPORT_TOL) fica com a queda
    // vertical. Se a menor penetracao e horizontal, o objeto e empurrado
    // para fora por esse eixo, para o lado do proprio centro, no maximo
    // DEPEN_SPEED*dt por passo. O empurrao passa por ResolveAABBMovement,
    // que deixa uma caixa SAIR do que ela penetra mas nao entrar em outra.
    // O jogador (a ultima caixa) nao conta: ele nao empurra objetos.
    //
    // Tambem nao empurra ninguem a caixa de um objeto INCLINADO ou em
    // movimento (tombando, assentando, caindo, ou parado com tilt torto):
    // a AABB de uma caixa girada cobre mais do que o corpo, e a "penetracao"
    // dela e artefato. Sem isto, o cubo tombando sobre a aresta (cuja AABB
    // desce alem do topo do apoio e cobre a quina) empurrava o proprio apoio
    // para tras e o tombo perdia a aresta.
    bool InflatedBox(const Instance& o)
    {
        if (o.body.motion == MOTION_TIP || o.body.motion == MOTION_SETTLE
            || o.body.motion == MOTION_FALL)
            return true;
        if (o.shape != SHAPE_BOX)
            return false;   // esfera e cilindro do peao: AABB exata (rigid.h)
        // Caixa parada: so a AABB das 24 poses do cubo (entradas 0 ou +-1
        // em tilt) e exata.
        for (int c = 0; c < 3; ++c)
            for (int r = 0; r < 3; ++r)
            {
                float a = std::fabs(o.tilt[c][r]);
                if (a > 1e-3f && a < 1.0f - 1e-3f)
                    return true;
            }
        return false;
    }

    void Depenetrate(Instance& inst, std::vector<AABB>& boxes,
                     const std::vector<int>& owner, int self, float dt)
    {
        AABB&     box  = boxes[self];
        glm::vec3 c    = AABBCenter(box);
        glm::vec3 push(0.0f);
        float     best = 0.0f;

        for (size_t j = 0; j + 1 < boxes.size(); ++j)
        {
            if ((int)j == self)
                continue;
            if (owner[j] >= 0 && g_Instances[owner[j]].grabbable
                && InflatedBox(g_Instances[owner[j]]))
                continue;
            const AABB& b = boxes[j];

            float ox = std::min(box.max.x, b.max.x) - std::max(box.min.x, b.min.x);
            float oy = std::min(box.max.y, b.max.y) - std::max(box.min.y, b.min.y);
            float oz = std::min(box.max.z, b.max.z) - std::max(box.min.z, b.min.z);
            if (ox <= 0.0f || oy <= PhysicsConfig::SUPPORT_TOL || oz <= 0.0f)
                continue;

            int   axis = (ox < oz) ? 0 : 2;
            float pen  = std::min(ox, oz);
            if (pen <= PhysicsConfig::DEPEN_MIN || oy < pen || pen <= best)
                continue;

            glm::vec3 cb = AABBCenter(b);
            float side = (c[axis] >= cb[axis]) ? 1.0f : -1.0f;

            best = pen;
            push = glm::vec3(0.0f);
            push[axis] = side * std::min(pen, PhysicsConfig::DEPEN_SPEED * dt);
        }

        if (best <= 0.0f)
            return;

        bool hit[3] = { false, false, false };
        glm::vec3 applied = ResolveAABBMovement(box, push, boxes, self, hit);
        inst.pos.x += applied.x;
        inst.pos.z += applied.z;
        box.min.x  += applied.x;  box.max.x += applied.x;
        box.min.z  += applied.z;  box.max.z += applied.z;
    }

    // Muda o estado de movimento e zera o relogio do estado.
    void SetMotion(BodyState& body, int motion)
    {
        if (body.motion != motion)
        {
            body.motion = motion;
            body.timer  = 0.0f;
        }
    }

    // -----------------------------------------------------------------------
    // Teste de equilibrio (M7), devolvido como SupportInfo para tip/roll.
    //
    // A area de apoio e o retangulo [fx0,fx1] x [fz0,fz1] formado pelas
    // regioes em que a base encosta em caixas com topo na altura do apoio.
    // O objeto esta em equilibrio se a projecao do centro de massa c cair
    // dentro dela, com a folga BALANCE_MARGIN.
    //
    // Fora dela, o ponto da area mais proximo do centro e
    //     p = (clamp(c.x, fx0, fx1), clamp(c.z, fz0, fz1))
    // e o vetor horizontal  overhang = c - p  diz para que lado e quanto o
    // centro esta pendurado: e a direcao em que o torque da gravidade
    // (r x m*g, com r = c - p) derruba o objeto. So entram os eixos que
    // passaram da folga, exatamente como a direcao de deslizamento do M7.
    //
    // O centro de massa e o centro da bbox local levado para mundo. Ele
    // coincide com o centro da AABB de colisao: a caixa transformada (ou o
    // cilindro do peao, ou a esfera) e simetrica em relacao ao proprio
    // centro, entao a AABB envolvente tambem e.
    // -----------------------------------------------------------------------
    void FillSupportInfo(const AABB& box, const std::vector<AABB>& boxes, int self,
                         bool supported, float height, int support_k,
                         SupportInfo* s)
    {
        s->supported  = supported;
        s->height     = height;
        s->box_index  = supported ? support_k : -1;
        s->fx0 = s->fx1 = s->fz0 = s->fz1 = 0.0f;
        s->unbalanced = false;
        s->overhang   = glm::vec3(0.0f);

        if (!supported)
            return;

        float fx0, fx1, fz0, fz1;
        if (!SupportFootprint(box, boxes, self, height, PhysicsConfig::SUPPORT_TOL,
                              &fx0, &fx1, &fz0, &fz1))
            return;

        s->fx0 = fx0; s->fx1 = fx1;
        s->fz0 = fz0; s->fz1 = fz1;

        glm::vec3 c = AABBCenter(box);
        const float mg = PhysicsConfig::BALANCE_MARGIN;

        float ox = 0.0f, oz = 0.0f;   // quanto o centro passa da area
        if (c.x < fx0 - mg || c.x > fx1 + mg) ox = c.x - Clamp(c.x, fx0, fx1);
        if (c.z < fz0 - mg || c.z > fz1 + mg) oz = c.z - Clamp(c.z, fz0, fz1);

        if (ox != 0.0f || oz != 0.0f)
        {
            s->unbalanced = true;
            s->overhang   = glm::vec3(ox, 0.0f, oz);
        }
    }

    // Deslizamento do M7: acelera horizontalmente para o lado em que o
    // centro esta pendurado, ate a base perder o contato. E o que acontece
    // com as flags do M8 desligadas, ou quando o tombo nao tem espaco.
    void Slide(Instance& inst, const glm::vec3& overhang, float dt)
    {
        float len = std::sqrt(overhang.x*overhang.x + overhang.z*overhang.z);
        if (len <= 0.0f)
            return;

        glm::vec3 dir(overhang.x/len, 0.0f, overhang.z/len);

        inst.vel.x += dir.x * PhysicsConfig::SLIDE_ACCEL * dt;
        inst.vel.z += dir.z * PhysicsConfig::SLIDE_ACCEL * dt;

        float sp = HorizontalSpeed(inst.vel);
        if (sp > PhysicsConfig::SLIDE_MAX_SPEED)
        {
            inst.vel.x *= PhysicsConfig::SLIDE_MAX_SPEED / sp;
            inst.vel.z *= PhysicsConfig::SLIDE_MAX_SPEED / sp;
        }
    }

    // Um passo de um objeto solto. "ctx.self" e a caixa dele em *ctx.boxes.
    void StepObject(Instance& inst, PhysicsContext& ctx, float dt, bool just_released)
    {
        std::vector<AABB>&      boxes = *ctx.boxes;
        const std::vector<int>& owner = *ctx.owner;
        const int               k     = ctx.self;
        BodyState&              body  = inst.body;

        const bool tipping = PhysicsConfig::ENABLE_TIPPING;
        const bool rolling = PhysicsConfig::ENABLE_ROLLING;

        // Estado do M8 que sobrou com a flag correspondente desligada: volta
        // para a fisica do M7 (cair e apoiar).
        if (!tipping && (body.motion == MOTION_TIP || body.motion == MOTION_SETTLE))
            SetMotion(body, MOTION_FALL);
        if (!rolling && body.motion == MOTION_ROLL)
            SetMotion(body, MOTION_REST);

        // -------------------------------------------------------------------
        // TIP: girando sobre a aresta. A gravidade ja esta no pendulo de
        // TipStep, entao aqui nao ha queda vertical. TipStep tambem confere
        // se a ARESTA continua apoiada (o jogador pode ter pegado o objeto
        // de baixo) e, se nao, solta para FALL com v = omega x (centro -
        // pivo). Isso nao da para conferir pela AABB do objeto: girando sobre
        // a aresta, a parte da base que fica para fora desce abaixo do topo
        // do apoio (min.y = pivo.y - L*sen(theta)) mesmo com o apoio ali.
        //
        // Um tombo pode durar indefinidamente: se o giro bateria numa parede
        // ou noutro objeto, o objeto fica ESCORADO em TIP (sem sono), e volta
        // a tombar se o obstaculo sair.
        // -------------------------------------------------------------------
        if (body.motion == MOTION_TIP)
        {
            TipStep(inst, ctx, dt);
            body.sleep = 0.0f;
            return;
        }

        // -------------------------------------------------------------------
        // SETTLE: assentando na pose estavel depois de pousar torto. Ao
        // terminar (SettleStep devolve o objeto em REST), quem rola e ainda
        // guarda velocidade de rolamento em inst.vel (RollOnLand no pouso)
        // continua rolando; os demais param.
        // -------------------------------------------------------------------
        if (body.motion == MOTION_SETTLE)
        {
            SettleStep(inst, ctx, dt);
            body.sleep = 0.0f;

            if (body.motion == MOTION_REST)
            {
                if (rolling && RollsNow(inst)
                    && HorizontalSpeed(inst.vel) > RollConfig::STOP_SPEED)
                {
                    SetMotion(body, MOTION_ROLL);
                }
                else
                {
                    inst.vel.x = 0.0f;
                    inst.vel.z = 0.0f;
                }
            }
            return;
        }

        // -------------------------------------------------------------------
        // REST / FALL / ROLL.
        // -------------------------------------------------------------------
        const int   prev_motion = body.motion;
        const float prev_vy     = inst.vel.y;

        AABB& box = boxes[k];

        if (tipping || rolling)
            Depenetrate(inst, boxes, owner, k, dt);

        // 1) Vertical: Euler semi-implicito, v(t+dt) = v(t) - g*dt e
        //    dy = v(t+dt)*dt. Se a queda deste passo passaria da superficie
        //    de apoio mais alta, o objeto para exatamente em cima dela.
        float v = inst.vel.y - PhysicsConfig::GRAVITY * dt;
        if (v < -PhysicsConfig::MAX_FALL_SPEED)
            v = -PhysicsConfig::MAX_FALL_SPEED;

        float dy        = v * dt;
        bool  supported = false;
        float support   = 0.0f;
        int   support_k = -1;

        if (FindSupportHeight(box, boxes, k, PhysicsConfig::SUPPORT_TOL,
                              &support, &support_k)
            && box.min.y + dy <= support)
        {
            supported = true;

            float snap_dy = support - box.min.y;

            // A cabeca do jogador em movimento nao empurra para cima um
            // objeto que ja estava apoiado em outra coisa.
            if (snap_dy > 0.0f && std::fabs(support - boxes.back().max.y) < 1e-4f)
                snap_dy = 0.0f;

            dy = snap_dy;

            // Se o apoio e um objeto que tambem esta caindo, herda a
            // velocidade dele: assim uma pilha solta no ar desce unida, em
            // vez de o de cima ficar um passo atras a cada quadro.
            int support_owner = (support_k >= 0) ? owner[support_k] : -1;
            if (support_owner >= 0 && g_Instances[support_owner].vel.y < 0.0f)
                v = g_Instances[support_owner].vel.y;
            else
                v = 0.0f;
        }
        else if (dy > 0.0f)
        {
            // Subindo (solto da mao com velocidade para cima): FindSupportHeight
            // so olha para baixo, entao a subida passa pela varredura por eixo
            // de ResolveAABBMovement. Se bateu no teto ou na base de algo, a
            // componente vertical da velocidade se perde (choque inelastico) e
            // a gravidade traz o objeto de volta.
            bool hit[3] = { false, false, false };
            dy = ResolveAABBMovement(box, glm::vec3(0.0f, dy, 0.0f), boxes, k, hit).y;
            if (hit[1])
                v = 0.0f;
        }

        inst.pos.y += dy;
        box.min.y  += dy;
        box.max.y  += dy;
        inst.vel.y  = v;

        // "Carona": apoiado num objeto que ainda esta caindo. Desce junto,
        // mas nao pousou de verdade, entao nao assenta nem tomba ainda.
        const bool riding = supported && v < 0.0f;

        SupportInfo sup;
        FillSupportInfo(box, boxes, k, supported, support, support_k, &sup);

        if (!supported)
        {
            // 2a) No ar: cai mantendo a velocidade horizontal (o objeto que
            //     saiu da borda continua indo para fora enquanto cai) e gira
            //     em torno do centro com a velocidade angular que trouxe do
            //     tombo ou do rolamento (momento angular conservado: sem
            //     torque no ar, omega constante).
            SetMotion(body, MOTION_FALL);
            body.support = -1;

            if ((tipping || rolling) && Length3(body.ang_vel) > 0.0f)
                AirSpinStep(inst, ctx, dt);
        }
        else
        {
            int so = (support_k >= 0) ? owner[support_k] : -1;
            body.support = (so >= 0 && g_Instances[so].grabbable) ? so : -1;

            // 2b) Pouso: estava no ar (ou descendo de carona, ou acabou de
            //     sair da mao) e agora esta apoiado e parado em Y.
            const bool landed = !riding
                && (prev_motion == MOTION_FALL || prev_vy < 0.0f || just_released);

            if (landed)
            {
                if (rolling && RollsNow(inst))
                {
                    // Bola ou peao deitado: o que sobrou do movimento vira
                    // rolamento (RollOnLand). Se o peao pousou torto, assenta
                    // antes, guardando a velocidade de rolamento em inst.vel.
                    RollOnLand(inst, ctx);
                    if (tipping && SettleBegin(inst, ctx))
                    {
                        body.sleep = 0.0f;
                        return;
                    }
                }
                else if (tipping)
                {
                    // Atrito do pouso: um objeto que nao rola para onde cai.
                    // Se a pose e torta, assenta numa face (SETTLE).
                    if (SettleBegin(inst, ctx))
                    {
                        inst.vel.x = 0.0f;
                        inst.vel.z = 0.0f;
                        body.sleep = 0.0f;
                        return;
                    }
                    SetMotion(body, MOTION_REST);
                    body.ang_vel = glm::vec3(0.0f);
                }
                else
                {
                    SetMotion(body, MOTION_REST);
                    body.ang_vel = glm::vec3(0.0f);
                }
            }

            // 3) e 4) Equilibrio.
            if (sup.unbalanced)
            {
                if (rolling && !riding && RollsNow(inst)
                    && RollHandlesOverhang(inst, sup.overhang))
                {
                    // Bola ou peao deitado com o centro alem da borda: sai
                    // rolando para fora (RollStep acelera para o lado do
                    // overhang).
                    SetMotion(body, MOTION_ROLL);
                    RollStep(inst, ctx, sup, dt);
                }
                else if (tipping && !riding && TipBegin(inst, ctx, sup))
                {
                    // Comeca a girar sobre a aresta; a partir do proximo
                    // passo quem move o objeto e TipStep.
                    body.sleep = 0.0f;
                    return;
                }
                else
                {
                    Slide(inst, sup.overhang, dt);
                }
            }
            else if (body.motion == MOTION_ROLL && !riding)
            {
                // Rolando sobre o apoio: atrito de rolamento (RollStep).
                RollStep(inst, ctx, sup, dt);
            }
            else
            {
                // Apoiado e em equilibrio: atrito estatico zera o movimento
                // horizontal.
                inst.vel.x = 0.0f;
                inst.vel.z = 0.0f;
            }
        }

        // 5) Horizontal, resolvido contra a cena um eixo por vez, para o
        //    objeto nao atravessar parede nem outro objeto. Quem rola gira
        //    pela distancia REALMENTE andada e quica no que bateu
        //    (RollAfterMove); os outros param no eixo da batida (M7).
        if (inst.vel.x != 0.0f || inst.vel.z != 0.0f)
        {
            glm::vec3 delta(inst.vel.x * dt, 0.0f, inst.vel.z * dt);
            bool hit[3] = { false, false, false };

            glm::vec3 applied = ResolveAABBMovement(box, delta, boxes, k, hit);

            inst.pos.x += applied.x;
            inst.pos.z += applied.z;
            box.min.x  += applied.x;  box.max.x += applied.x;
            box.min.z  += applied.z;  box.max.z += applied.z;

            const bool roller = rolling && RollsNow(inst)
                && (body.motion == MOTION_ROLL || body.motion == MOTION_FALL);

            if (roller)
            {
                RollAfterMove(inst, ctx, applied, hit, dt);
            }
            else
            {
                if (hit[0]) inst.vel.x = 0.0f;
                if (hit[2]) inst.vel.z = 0.0f;
            }
        }

        // 6) Sono. Apoiado, equilibrado e quase parado por SLEEP_TIME: zera
        //    as velocidades. Sem isto, um objeto rolando bem devagar ficaria
        //    acumulando rotacoes minusculas (e erro de arredondamento) para
        //    sempre.
        const bool may_sleep = supported && !riding && !sup.unbalanced
            && (body.motion == MOTION_REST || body.motion == MOTION_ROLL);

        if (may_sleep
            && Length3(inst.vel) < PhysicsConfig::SLEEP_LINEAR
            && Length3(body.ang_vel) < PhysicsConfig::SLEEP_ANGULAR)
        {
            body.sleep += dt;
            if (body.sleep >= PhysicsConfig::SLEEP_TIME)
            {
                inst.vel     = glm::vec3(0.0f);
                body.ang_vel = glm::vec3(0.0f);
                SetMotion(body, MOTION_REST);
            }
        }
        else
        {
            body.sleep = 0.0f;
        }

        if (body.motion == prev_motion)
            body.timer += dt;
    }
}

void UpdateFallingObjects(int held_instance, const AABB& player_box, float dt)
{
    if (dt <= 0.0f)
        return;

    // Objeto que estava na mao no passo anterior e acabou de ser solto: o
    // primeiro passo dele conta como pouso, para que um objeto pego no meio
    // de um tombo e largado ja encostado em algo ainda assente numa face.
    static int s_prev_held = -1;
    const int just_released = (s_prev_held >= 0 && s_prev_held != held_instance)
                            ? s_prev_held : -1;
    s_prev_held = held_instance;

    // Objeto na mao: nao cai e nao gira. A orientacao (tilt) fica como esta.
    if (held_instance >= 0 && held_instance < (int)g_Instances.size())
    {
        g_Instances[held_instance].vel  = glm::vec3(0.0f);
        g_Instances[held_instance].body = BodyState();
    }

    // -----------------------------------------------------------------------
    // 1) AABBs em mundo de tudo que e solido, mais o jogador (a ULTIMA).
    //
    // A lista e montada uma vez por passo e atualizada a cada objeto movido
    // (aqui e em tip.cpp/roll.cpp), para que dois objetos caindo um sobre o
    // outro no mesmo passo se vejam nas posicoes corretas.
    // -----------------------------------------------------------------------
    std::vector<AABB>      boxes;
    std::vector<int>       owner;   // instancia dona de cada caixa (-1 = jogador)
    std::vector<glm::vec3> lmins, lmaxs;
    boxes.reserve(g_Instances.size() + 1);
    owner.reserve(g_Instances.size() + 1);
    lmins.reserve(g_Instances.size() + 1);
    lmaxs.reserve(g_Instances.size() + 1);

    for (size_t i = 0; i < g_Instances.size(); ++i)
    {
        const Instance& inst = g_Instances[i];
        if (!inst.solid)
            continue;

        glm::vec3 lmin, lmax;
        if (!GetMeshLocalBBox(inst.mesh, &lmin, &lmax))
            continue;

        boxes.push_back(InstanceCollisionBox((int)i, lmin, lmax));
        owner.push_back((int)i);
        lmins.push_back(lmin);
        lmaxs.push_back(lmax);
    }

    boxes.push_back(player_box);
    owner.push_back(-1);
    lmins.push_back(glm::vec3(0.0f));
    lmaxs.push_back(glm::vec3(0.0f));

    // -----------------------------------------------------------------------
    // 2) Objetos pegaveis, do mais baixo para o mais alto (min.y crescente).
    //    Numa pilha, o de baixo e resolvido primeiro e o de cima enxerga a
    //    posicao nova dele no mesmo passo.
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

        if (idx == held_instance)
            continue;

        PhysicsContext ctx;
        ctx.boxes    = &boxes;
        ctx.owner    = &owner;
        ctx.self     = (int)k;
        ctx.instance = idx;
        ctx.lmin     = lmins[k];
        ctx.lmax     = lmaxs[k];

        StepObject(g_Instances[idx], ctx, dt, idx == just_released);
    }
}
