// Tombar (M8): giro sobre a aresta do apoio, rotacao livre no ar e
// assentamento numa pose estavel. Interface e ciclo de estados em tip.h.
//
// Tudo e escrito a mao (produto escalar, vetorial, normas) porque os helpers
// de vetor da GLM sao vetados pelo enunciado; as rotacoes passam por rigid.h
// (Rodrigues).

#include "tip.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "physics.h"   // PhysicsConfig::GRAVITY, SUPPORT_TOL
#include "roll.h"      // RollsNow, RollConfig::STOP_SPEED

namespace
{
    const glm::vec3 UP(0.0f, 1.0f, 0.0f);

    // Mesma folga de contato de collisions.cpp (ResolveSingleAxis,
    // FindSupportHeight): caixas que apenas se encostam face a face NAO
    // estao se interpenetrando. Sem ela, um cubo encostado de lado em outro,
    // ou outro objeto pousado em cima dele, bloquearia qualquer giro.
    const float CONTACT_EPS = 1e-4f;

    float Dot(const glm::vec3& a, const glm::vec3& b)
    {
        return a.x*b.x + a.y*b.y + a.z*b.z;
    }

    glm::vec3 Cross(const glm::vec3& a, const glm::vec3& b)
    {
        return glm::vec3(a.y*b.z - a.z*b.y,
                         a.z*b.x - a.x*b.z,
                         a.x*b.y - a.y*b.x);
    }

    float Len(const glm::vec3& a)
    {
        return std::sqrt(Dot(a, a));
    }

    // Interpenetracao ESTRITA de duas AABBs: precisam se sobrepor mais que
    // CONTACT_EPS nos tres eixos. (AABBOverlap de collisions.cpp conta o
    // simples encostar como sobreposicao, o que aqui seria falso positivo.)
    bool Penetrates(const AABB& a, const AABB& b)
    {
        for (int i = 0; i < 3; ++i)
        {
            if (a.max[i] <= b.min[i] + CONTACT_EPS || a.min[i] >= b.max[i] - CONTACT_EPS)
                return false;
        }
        return true;
    }

    AABB BoxOf(const Instance& inst, const PhysicsContext& ctx)
    {
        return InstanceCollisionBox(inst, ctx.instance, ctx.lmin, ctx.lmax);
    }

    // Toda mudanca de pose atualiza a caixa deste objeto na lista do passo,
    // para os objetos resolvidos depois dele enxergarem a posicao nova.
    void StoreBox(const Instance& inst, PhysicsContext& ctx)
    {
        (*ctx.boxes)[ctx.self] = BoxOf(inst, ctx);
    }

    // Copia so a pose (posicao e inclinacao) de "from" para "to". As funcoes
    // abaixo testam o giro numa copia da instancia e so depois o aplicam.
    void CopyPose(Instance& to, const Instance& from)
    {
        to.pos  = from.pos;
        to.tilt = from.tilt;
        to.yaw  = from.yaw;
    }

    // Verdadeiro se "nb" (caixa nova) entra em alguma caixa da cena em que a
    // caixa antiga "ob" ainda nao estava entrando. O proprio objeto e
    // ignorado. Sobreposicoes que ja existiam sao toleradas de proposito:
    // logo depois de soltar da aresta, a AABB do objeto inclinado ainda
    // cobre o canto do apoio antigo (a AABB e maior que o corpo), e isso
    // nao pode travar o giro no ar.
    bool NewPenetration(const AABB& ob, const AABB& nb, const PhysicsContext& ctx)
    {
        const std::vector<AABB>& boxes = *ctx.boxes;
        for (size_t k = 0; k < boxes.size(); ++k)
        {
            if ((int)k == ctx.self)
                continue;
            if (Penetrates(nb, boxes[k]) && !Penetrates(ob, boxes[k]))
                return true;
        }
        return false;
    }

    // -----------------------------------------------------------------------
    // Caixas que fazem parte do APOIO de um tombo sobre a aresta (pivot,
    // direcao para fora d). Girando sobre a aresta, a parte de fora da base
    // desce abaixo do topo do apoio, e a AABB do objeto (que e uma caixa
    // envolvente) passa a cobrir o canto do apoio sem que o corpo de verdade
    // o atravesse. Por isso o apoio fica fora do teste de colisao do tombo.
    //
    // Uma caixa e apoio se o seu topo esta na altura do pivo (com a mesma
    // tolerancia do teste de apoio) e ela comeca do lado de DENTRO da aresta:
    //     min_{p na caixa} (p - pivot) . d  <=  0
    // Isso pega tambem a segunda caixa de um objeto apoiado em duas (ponte),
    // que fica toda do lado de dentro. Uma caixa na mesma altura mas do lado
    // de FORA da aresta nao e apoio: e um obstaculo em que a base pode bater.
    // -----------------------------------------------------------------------
    bool IsTipSupport(const AABB& b, const glm::vec3& pivot, const glm::vec3& d)
    {
        if (std::fabs(b.max.y - pivot.y) > PhysicsConfig::SUPPORT_TOL)
            return false;

        float lo = 0.0f; // menor valor de p . d sobre a caixa
        for (int i = 0; i < 3; ++i)
            lo += (d[i] > 0.0f) ? b.min[i]*d[i] : b.max[i]*d[i];

        return lo - Dot(pivot, d) <= CONTACT_EPS;
    }

    // Ainda existe apoio sob a aresta? Alguma caixa com o topo na altura do
    // pivo e o pivo dentro dela no plano XZ. Se o jogador pegou o objeto de
    // baixo, nao ha mais aresta para girar e o tombo vira queda.
    bool PivotSupported(const PhysicsContext& ctx, const glm::vec3& pivot)
    {
        const std::vector<AABB>& boxes = *ctx.boxes;
        const float E = 1e-3f;
        for (size_t k = 0; k < boxes.size(); ++k)
        {
            if ((int)k == ctx.self)
                continue;
            const AABB& b = boxes[k];
            if (std::fabs(b.max.y - pivot.y) > PhysicsConfig::SUPPORT_TOL)
                continue;
            if (pivot.x < b.min.x - E || pivot.x > b.max.x + E)
                continue;
            if (pivot.z < b.min.z - E || pivot.z > b.max.z + E)
                continue;
            return true;
        }
        return false;
    }

    // A caixa nova do tombo entra em algo que nao e o proprio objeto nem o
    // apoio (parede, outro objeto, o jogador = ultima caixa)?
    bool TipBlocked(const AABB& nb, const PhysicsContext& ctx,
                    const glm::vec3& pivot, const glm::vec3& d)
    {
        const std::vector<AABB>& boxes = *ctx.boxes;
        for (size_t k = 0; k < boxes.size(); ++k)
        {
            if ((int)k == ctx.self)
                continue;
            if (IsTipSupport(boxes[k], pivot, d))
                continue;
            if (Penetrates(nb, boxes[k]))
                return true;
        }
        return false;
    }

    // -----------------------------------------------------------------------
    // Momento de inercia POR UNIDADE DE MASSA em torno de um eixo unitario n
    // que passa pelo centro, aproximando o corpo por uma caixa macica de
    // meias-arestas (a, b, c) nos eixos locais. Nos eixos principais da caixa
    //     I_xx = m (b^2 + c^2) / 3,  I_yy = m (a^2 + c^2) / 3,  I_zz = m (a^2 + b^2) / 3
    // (lado 2a: m (2b)^2/12 + m (2c)^2/12 = m (b^2 + c^2)/3), e para um eixo
    // qualquer  I_n = n^T I n = sum_i I_ii n_i^2,  com n_i = n . e_i a
    // componente de n no eixo local i (e_i = coluna i da orientacao).
    // Assim o yaw e uma inclinacao anterior entram certo. A massa cancela
    // na equacao do movimento, entao nunca aparece.
    // -----------------------------------------------------------------------
    float InertiaPerMass(const Instance& inst, const PhysicsContext& ctx, const glm::vec3& n)
    {
        glm::mat4 O = InstanceOrientation(inst);
        float h[3];
        for (int i = 0; i < 3; ++i)
            h[i] = 0.5f * (ctx.lmax[i] - ctx.lmin[i]) * inst.scale[i];

        float I = 0.0f;
        for (int i = 0; i < 3; ++i)
        {
            glm::vec3 e(O[i][0], O[i][1], O[i][2]);
            float el = Len(e);
            float ni = (el > 0.0f) ? Dot(n, e) / el : 0.0f;
            int j = (i + 1) % 3, k = (i + 2) % 3;
            I += ni*ni * (h[j]*h[j] + h[k]*h[k]) / 3.0f;
        }
        return I;
    }

    // Maior meia-aresta do corpo em metros (escala ja aplicada).
    float MaxHalfExtent(const Instance& inst, const PhysicsContext& ctx)
    {
        float m = 0.0f;
        for (int i = 0; i < 3; ++i)
            m = std::max(m, 0.5f * (ctx.lmax[i] - ctx.lmin[i]) * std::fabs(inst.scale[i]));
        return m;
    }

    // Fim do tombo: o objeto larga a aresta e continua o movimento que tinha.
    // Girando com omega em torno da reta fixa (pivot, axis), cada ponto do
    // corpo tem velocidade  v = w x (p - pivot), w = omega * axis. O centro
    // sai com essa velocidade e o corpo segue girando com a mesma w em torno
    // do proprio centro (AirSpinStep), agora so sob a gravidade.
    void ReleaseTip(Instance& inst, PhysicsContext& ctx, float omega)
    {
        BodyState& b = inst.body;
        glm::vec3 w = b.axis * omega;
        glm::vec3 c = InstanceBodyCenter(inst, ctx.lmin, ctx.lmax);

        inst.vel  = Cross(w, c - b.pivot);
        b.ang_vel = w;
        b.motion  = MOTION_FALL;
        b.angle   = 0.0f;
        b.timer   = 0.0f;
        StoreBox(inst, ctx);
    }

    // -----------------------------------------------------------------------
    // Reapoio da base depois de girar em torno do centro (SETTLE).
    //
    // Antes do giro a base estava encostada no apoio (old_min_y). Ao
    // endireitar, a AABB encolhe e a base sobe; o objeto tem que descer ate
    // encostar de novo. Procuramos o apoio com a pegada XZ da pose NOVA, mas
    // com a base na altura ANTIGA: assim o apoio de verdade e achado mesmo
    // que o giro tenha feito a AABB crescer para baixo e entrar nele mais
    // que SUPPORT_TOL (o que faria FindSupportHeight pular para o piso).
    //
    // Devolve false se, sob a pegada nova, nao ha mais nada na altura do
    // apoio antigo (o objeto endireitou para fora da borda): ai ele cai.
    // -----------------------------------------------------------------------
    bool Resupport(Instance& inst, const PhysicsContext& ctx, float old_min_y, AABB* out_box)
    {
        AABB nb = BoxOf(inst, ctx);
        AABB q  = nb;
        q.min.y = old_min_y;

        float h = 0.0f;
        int   k = -1;
        if (!FindSupportHeight(q, *ctx.boxes, ctx.self, PhysicsConfig::SUPPORT_TOL, &h, &k))
            return false;
        if (h < old_min_y - PhysicsConfig::SUPPORT_TOL)
            return false;

        float dy = h - nb.min.y;
        inst.pos.y += dy;
        nb.min.y   += dy;
        nb.max.y   += dy;
        if (out_box)
            *out_box = nb;
        return true;
    }

    // Duracao total do assentamento. O tempo de queda de um corpo rigido
    // girando sob a gravidade escala com sqrt(L/g) (pendulo), entao um dado
    // de 1,5 m leva ~2x o tempo de um cubo de 0,36 m para deitar. A
    // referencia SETTLE_TIME vale para meia-aresta SETTLE_REF_HALF.
    float SettleDuration(const Instance& inst, const PhysicsContext& ctx)
    {
        float f = std::sqrt(MaxHalfExtent(inst, ctx) / TipConfig::SETTLE_REF_HALF);
        f = std::max(TipConfig::SETTLE_SCALE_MIN, std::min(TipConfig::SETTLE_SCALE_MAX, f));
        return TipConfig::SETTLE_TIME * f;
    }
}

// ---------------------------------------------------------------------------
// TipBegin
//
// O apoio e uma AABB, entao a borda da area de apoio e alinhada aos eixos do
// mundo. O eixo dominante do overhang diz por qual aresta o objeto passa:
// overhang.x > 0 -> aresta x = fx1, direcao para fora d = (+1, 0, 0), etc.
// O pivo e o ponto dessa aresta na altura do apoio, na coordenada
// transversal do centro (limitada a area de apoio).
//
// Eixo de giro:  axis = up x d.  Pela regra da mao direita, um angulo
// positivo em torno dele leva up para d (Rodrigues com axis _|_ up:
// R*up = up cos(t) + (axis x up) sin(t) = up cos(t) + d sin(t)), ou seja,
// o topo vai para fora: e o sentido do tombo.
// ---------------------------------------------------------------------------
bool TipBegin(Instance& inst, PhysicsContext& ctx, const SupportInfo& sup)
{
    if (!sup.supported)
        return false;

    const float ox = sup.overhang.x;
    const float oz = sup.overhang.z;
    if (std::fabs(ox) < 1e-6f && std::fabs(oz) < 1e-6f)
        return false;

    glm::vec3 c = InstanceBodyCenter(inst, ctx.lmin, ctx.lmax);
    glm::vec3 d(0.0f);
    glm::vec3 pivot(0.0f, sup.height, 0.0f);

    if (std::fabs(ox) >= std::fabs(oz))
    {
        d.x     = (ox > 0.0f) ? 1.0f : -1.0f;
        pivot.x = (ox > 0.0f) ? sup.fx1 : sup.fx0;
        pivot.z = std::max(sup.fz0, std::min(sup.fz1, c.z));
    }
    else
    {
        d.z     = (oz > 0.0f) ? 1.0f : -1.0f;
        pivot.z = (oz > 0.0f) ? sup.fz1 : sup.fz0;
        pivot.x = std::max(sup.fx0, std::min(sup.fx1, c.x));
    }

    // Braco inicial da gravidade: o centro tem que estar para fora da aresta.
    if (Dot(c - pivot, d) <= 0.0f)
        return false;

    glm::vec3 axis = Cross(UP, d);

    // Ha espaco para girar? Testa um giro pequeno (PROBE_ANGLE) sobre a
    // aresta: se ja ai a caixa entraria numa parede, em outro objeto ou no
    // jogador, nao tomba e physics.cpp cai no deslizamento do M7 (que para
    // no obstaculo). E o caso do cubo pendurado com a face encostada na
    // parede: na vida real ele fica escorado nela.
    Instance probe = inst;
    RotateAboutLine(probe, ctx.lmin, ctx.lmax, pivot, axis, TipConfig::PROBE_ANGLE);
    if (TipBlocked(BoxOf(probe, ctx), ctx, pivot, d))
        return false;

    BodyState& b = inst.body;
    b.motion  = MOTION_TIP;
    b.pivot   = pivot;
    b.axis    = axis;
    b.angle   = 0.0f;
    b.timer   = 0.0f;
    b.ang_vel = glm::vec3(0.0f);
    inst.vel  = glm::vec3(0.0f);
    return true;
}

// ---------------------------------------------------------------------------
// TipStep: corpo rigido girando em torno de um eixo FIXO (a aresta).
//
// Torque da gravidade em relacao a aresta, com r = centro - pivo e
// d = axis x up a direcao para fora:
//     tau = ((r x (-m g up)) . axis) = m g (r . d)
// (produto misto: (r x up) . axis = r . (up x axis) = -r . d). So o braco
// HORIZONTAL r . d importa. Pela 2a lei de Newton para rotacao,
//     I_pivo * alpha = m g (r . d)
// com o teorema dos eixos paralelos (Steiner)
//     I_pivo = I_cm + m dist^2,   dist^2 = |r|^2 - (r . axis)^2
// (dist = distancia do centro a reta do pivo). A massa cancela:
//     alpha = g (r . d) / (I_cm/m + dist^2)
// Integracao por Euler semi-implicito, como no resto da fisica:
//     omega += alpha dt ;  dtheta = omega dt
// e RotateAboutLine gira o corpo dtheta em torno da aresta.
//
// O braco cresce com o angulo (o centro vai para fora e para baixo), entao o
// tombo comeca devagar e acelera, como na vida real.
//
// Quando soltar (ReleaseTip):
//  1) separacao da face: a face de baixo, inclinada de theta, so continua
//     apoiada na aresta enquanto o apoio precisa empurra-la. Sem a forca de
//     contato o ponto do corpo sobre a aresta teria aceleracao
//         a_P = -g up + omega^2 r          (queda livre + centripeta)
//     e, com n a normal da face (para fora do corpo) e h a distancia do
//     centro ao plano da face, a componente  a_P . n = g cos(theta) -
//     omega^2 h  diz se a face avancaria sobre a aresta (> 0, contato
//     continua) ou se afastaria dela (<= 0, solta). Condicao de soltar:
//         omega^2 h >= g cos(theta)
//     Para um cubo isso da ~52-57 graus, conforme o quanto passou da borda;
//  2) seguranca: theta >= RELEASE_ANGLE ou timer >= MAX_TIP_TIME;
//  3) o giro do passo faria a caixa entrar em algo que nao e o apoio
//     (parede, outro objeto, jogador): o passo nao e aplicado e solta.
// ---------------------------------------------------------------------------
void TipStep(Instance& inst, PhysicsContext& ctx, float dt)
{
    BodyState& b = inst.body;
    b.timer += dt;

    const glm::vec3 axis  = b.axis;
    const glm::vec3 pivot = b.pivot;
    const glm::vec3 d     = Cross(axis, UP);   // direcao para fora
    const float     g     = PhysicsConfig::GRAVITY;

    // omega fica guardado em ang_vel (pedido de physics.cpp: ang_vel sempre
    // valendo omega * axis durante o tombo).
    float omega = Dot(b.ang_vel, axis);

    // O apoio sumiu (pegaram o objeto de baixo): larga a aresta com o
    // movimento que tinha. Feito aqui, e nao em physics.cpp, porque a AABB do
    // objeto inclinado desce abaixo do topo do apoio e um teste de apoio pela
    // AABB (FindSupportHeight) nao enxerga mais a aresta.
    if (!PivotSupported(ctx, pivot))
    {
        ReleaseTip(inst, ctx, omega);
        return;
    }

    glm::vec3 c = InstanceBodyCenter(inst, ctx.lmin, ctx.lmax);
    glm::vec3 r = c - pivot;

    float ra    = Dot(r, axis);
    float dist2 = std::max(0.0f, Dot(r, r) - ra*ra);
    float Ip    = InertiaPerMass(inst, ctx, axis) + dist2;   // I_pivo / m
    if (Ip < 1e-8f)
        Ip = 1e-8f;

    float alpha  = g * Dot(r, d) / Ip;
    omega       += alpha * dt;
    float dtheta = omega * dt;

    // Voltou para dentro (so com braco inicial ~0 por erro numerico): desfaz
    // o que girou e fica parado na pose original.
    if (b.angle + dtheta <= 0.0f)
    {
        RotateAboutLine(inst, ctx.lmin, ctx.lmax, pivot, axis, -b.angle);
        b.angle   = 0.0f;
        b.ang_vel = glm::vec3(0.0f);
        b.motion  = MOTION_REST;
        b.timer   = 0.0f;
        StoreBox(inst, ctx);
        return;
    }

    Instance next = inst;
    RotateAboutLine(next, ctx.lmin, ctx.lmax, pivot, axis, dtheta);
    if (TipBlocked(BoxOf(next, ctx), ctx, pivot, d))
    {
        // Bateu (parede, outro objeto, jogador): choque sem quique. Gira so
        // ate encostar (bissecao da fracao do passo que ainda cabe), zera
        // omega e fica ESCORADO, ainda em TIP: o apoio segura a base e o
        // obstaculo segura o topo, como um cubo tombado contra a parede. Se
        // o obstaculo sair (o jogador se afasta), o proximo passo volta a
        // acelerar sozinho. Soltar aqui faria o objeto, com a base mal
        // abaixo do topo do apoio, ser reapoiado mais alto por physics.cpp.
        float lo = 0.0f, hi = dtheta;
        for (int it = 0; it < 10; ++it)
        {
            float mid = 0.5f * (lo + hi);
            Instance t = inst;
            RotateAboutLine(t, ctx.lmin, ctx.lmax, pivot, axis, mid);
            if (TipBlocked(BoxOf(t, ctx), ctx, pivot, d)) hi = mid; else lo = mid;
        }
        if (lo > 0.0f)
        {
            RotateAboutLine(inst, ctx.lmin, ctx.lmax, pivot, axis, lo);
            b.angle += lo;
        }
        b.ang_vel = glm::vec3(0.0f);
        b.timer   = 0.0f;   // MAX_TIP_TIME conta so o tombo livre
        StoreBox(inst, ctx);
        return;
    }

    CopyPose(inst, next);
    b.angle  += dtheta;
    b.ang_vel = axis * omega;
    StoreBox(inst, ctx);

    // Separacao da face (ver acima). n = -(up cos(theta) + d sin(theta)),
    // h = -(r . n) = (r . up) cos(theta) + (r . d) sin(theta).
    glm::vec3 r1 = InstanceBodyCenter(inst, ctx.lmin, ctx.lmax) - pivot;
    float ct = std::cos(b.angle), st = std::sin(b.angle);
    float h  = Dot(r1, UP) * ct + Dot(r1, d) * st;

    // So solta pela separacao quando a base ja desceu mais que SUPPORT_TOL
    // abaixo do topo do apoio. Antes disso physics.cpp ainda acharia o
    // apoio antigo "logo abaixo" e levantaria o objeto ate ele (so importa
    // para objetos de poucos centimetros; um cubo de 0,36 m solta com a base
    // ~17 cm abaixo).
    bool below = (*ctx.boxes)[ctx.self].min.y < pivot.y - PhysicsConfig::SUPPORT_TOL - 1e-3f;
    bool separates = below && (omega * omega * h >= g * ct);
    if (separates || b.angle >= TipConfig::RELEASE_ANGLE || b.timer >= TipConfig::MAX_TIP_TIME)
        ReleaseTip(inst, ctx, omega);
}

// ---------------------------------------------------------------------------
// AirSpinStep: no ar a gravidade nao faz torque em relacao ao centro de
// massa, entao a velocidade angular se conserva (a menos do amortecimento
// opcional AIR_SPIN_DAMP) e o corpo gira em torno do proprio centro:
//     theta = |w| dt  em torno de  w / |w|
// Se a caixa nova entraria em algo em que a antiga nao estava entrando, o
// giro deste passo nao e aplicado, mas w e mantido: a AABB e maior que o
// corpo (um peao caindo rente a lateral do dado de onde tombou "bate" so
// pela caixa), e zerar w ali congelava o objeto quase em pe, e ele
// assentava em pe em vez de deitar. Assim que a caixa sai do obstaculo
// (o objeto segue caindo e indo para fora), o giro continua. No pouso
// SettleBegin/RollOnLand assumem a rotacao.
// ---------------------------------------------------------------------------
void AirSpinStep(Instance& inst, PhysicsContext& ctx, float dt)
{
    glm::vec3& w = inst.body.ang_vel;

    if (TipConfig::AIR_SPIN_DAMP > 0.0f)
        w = w * std::max(0.0f, 1.0f - TipConfig::AIR_SPIN_DAMP * dt);

    float wl = Len(w);
    if (wl < 1e-6f)
    {
        w = glm::vec3(0.0f);
        return;
    }

    AABB ob = BoxOf(inst, ctx);
    Instance next = inst;
    RotateAboutCenter(next, ctx.lmin, ctx.lmax, w * (1.0f / wl), wl * dt);
    AABB nb = BoxOf(next, ctx);

    if (NewPenetration(ob, nb, ctx))
    {
        (*ctx.boxes)[ctx.self] = ob;
        return;
    }

    CopyPose(inst, next);
    (*ctx.boxes)[ctx.self] = nb;
}

// ---------------------------------------------------------------------------
// SettleBegin: o objeto acabou de pousar. Se a orientacao nao e uma pose
// estavel (rigid.h), guarda o eixo e o angulo que faltam e passa a SETTLE.
// Se ja e (a menos de STABLE_TOL), so limpa o erro numerico com
// CanonicalizePose e reapoia a base, porque a AABB pode mudar alguns mm.
// Nao mexe em inst.vel (physics.cpp guarda nela a velocidade de rolamento).
// ---------------------------------------------------------------------------
bool SettleBegin(Instance& inst, PhysicsContext& ctx)
{
    BodyState& b = inst.body;

    glm::vec3 axis(0.0f, 1.0f, 0.0f);
    float     angle = 0.0f;
    bool tilted = NearestStablePose(inst, TipConfig::STABLE_TOL, &axis, &angle);

    if (!tilted || !(angle > 0.0f))
    {
        float old_min_y = BoxOf(inst, ctx).min.y;
        CanonicalizePose(inst);
        Resupport(inst, ctx, old_min_y, 0);
        StoreBox(inst, ctx);
        return false;
    }

    b.motion  = MOTION_SETTLE;
    b.axis    = axis;
    b.angle   = angle;
    b.timer   = 0.0f;
    b.ang_vel = glm::vec3(0.0f);
    return true;
}

// ---------------------------------------------------------------------------
// SettleStep: gira em torno do centro ate a pose estavel e, a cada passo,
// reapoia a base (Resupport).
//
// Perfil do giro: em vez de velocidade constante, o angulo que falta segue
//     resto(t) = theta0 (1 - (t/T)^2)
// (aceleracao angular constante a partir do repouso), que e como um corpo
// cai deitando sobre a face: comeca devagar e chega com velocidade. Como so
// guardamos o resto e o tempo, o passo sai da razao
//     resto(t+dt) = resto(t) * (1 - ((t+dt)/T)^2) / (1 - (t/T)^2)
//
// Casos especiais:
//   - sem apoio sob a pegada nova (endireitou para fora da borda): passa a
//     FALL girando com a mesma velocidade angular; ao pousar, assenta de
//     novo;
//   - o giro faria a caixa entrar em algo (parede, outro objeto): para onde
//     esta, escorado, em REST (sem forcar a pose canonica).
// No fim: CanonicalizePose, ang_vel = 0, e ROLL se o objeto rola nesta pose
// (peao deitado) e ainda tem velocidade, senao REST.
// ---------------------------------------------------------------------------
void SettleStep(Instance& inst, PhysicsContext& ctx, float dt)
{
    BodyState& b = inst.body;

    const float T  = SettleDuration(inst, ctx);
    const float t0 = b.timer;
    const float t1 = b.timer + dt;
    b.timer = t1;

    bool  last = (t1 >= T) || (b.angle <= 1e-5f);
    float step = b.angle;
    if (!last)
    {
        float f0 = 1.0f - (t0/T)*(t0/T);
        float f1 = 1.0f - (t1/T)*(t1/T);
        step = (f0 > 1e-6f) ? b.angle * (1.0f - f1/f0) : b.angle;
    }

    AABB ob = BoxOf(inst, ctx);

    Instance next = inst;
    RotateAboutCenter(next, ctx.lmin, ctx.lmax, b.axis, step);
    if (last)
        CanonicalizePose(next);

    AABB nb;
    if (!Resupport(next, ctx, ob.min.y, &nb))
    {
        // Perdeu o apoio: cai, continuando o giro no ar.
        b.motion  = MOTION_FALL;
        b.ang_vel = b.axis * ((dt > 0.0f) ? step / dt : 0.0f);
        b.angle   = 0.0f;
        b.timer   = 0.0f;
        StoreBox(inst, ctx);
        return;
    }

    if (NewPenetration(ob, nb, ctx))
    {
        // Escorado em algo: fica como esta.
        b.motion  = MOTION_REST;
        b.ang_vel = glm::vec3(0.0f);
        b.angle   = 0.0f;
        b.timer   = 0.0f;
        StoreBox(inst, ctx);
        return;
    }

    CopyPose(inst, next);
    (*ctx.boxes)[ctx.self] = nb;

    if (!last)
    {
        b.angle  -= step;
        b.ang_vel = b.axis * ((dt > 0.0f) ? step / dt : 0.0f);
        return;
    }

    b.angle   = 0.0f;
    b.timer   = 0.0f;
    b.ang_vel = glm::vec3(0.0f);

    float sp = std::sqrt(inst.vel.x*inst.vel.x + inst.vel.z*inst.vel.z);
    b.motion = (RollsNow(inst) && sp > RollConfig::STOP_SPEED) ? MOTION_ROLL : MOTION_REST;
}
