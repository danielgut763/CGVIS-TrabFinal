// Cenarios de ponta a ponta do M8 (tombar, rolar, passo fixo), com a cena
// real (BuildScene) e a fisica avancando em passos de PhysicsConfig::FIXED_STEP
// como no main.cpp (acumulador de tempo por quadro). Ver tools/check.sh.
//
// Compilado normal: cenarios a-f, h-j (flags do M8 ligadas).
// Compilado com -DPHYSICS_ENABLE_TIPPING=0 -DPHYSICS_ENABLE_ROLLING=0:
// so o cenario g (comparacao com os numeros do M7).
//
// Devolve o numero de cenarios que falharam.
#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "scene.h"
#include "physics.h"
#include "level.h"
#include "player.h"
#include "grab.h"
#include "rigid.h"
#include "tip.h"
#include "roll.h"
// GetMeshLocalBBox vem de test/stub.cpp

static LevelState L;
static Player     P;
static int        g_fails = 0;
static int        g_held  = -1;
static float      g_acc   = 0.0f;
static const float FS     = PhysicsConfig::FIXED_STEP;
static const PlayerInput NONE = {0,0,0,0,0};

static const char* MotionName(int m)
{
    switch (m)
    {
        case MOTION_REST:   return "REST";
        case MOTION_FALL:   return "FALL";
        case MOTION_TIP:    return "TIP";
        case MOTION_SETTLE: return "SETTLE";
        case MOTION_ROLL:   return "ROLL";
    }
    return "?";
}

static void Verdict(const char* id, bool ok, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));
static void Verdict(const char* id, bool ok, const char* fmt, ...)
{
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("[%s] %-3s %s\n", ok ? "OK" : "FALHOU", id, buf);
    if (!ok) ++g_fails;
}

// ---------------------------------------------------------------------------
// Utilidades de cena.
// ---------------------------------------------------------------------------
static int FindMesh(const char* mesh, int nth = 0, bool grabbable_only = true)
{
    for (size_t i = 0; i < g_Instances.size(); ++i)
        if (g_Instances[i].mesh == mesh && (!grabbable_only || g_Instances[i].grabbable))
            if (nth-- == 0) return (int)i;
    return -1;
}

static AABB BoxOf(int i)
{
    glm::vec3 a, b;
    GetMeshLocalBBox(g_Instances[i].mesh, &a, &b);
    return InstanceCollisionBox(i, a, b);
}

static std::vector<AABB> SolidBoxes()
{
    std::vector<AABB> v;
    for (size_t i = 0; i < g_Instances.size(); ++i)
        if (g_Instances[i].solid) v.push_back(BoxOf((int)i));
    return v;
}

static void Reset()
{
    BuildScene(g_Instances);
    LevelInit(L);
    // Jogador num canto livre, longe de todos os cenarios.
    PlayerInit(P, glm::vec3(-5.3f, 0.0f, -3.8f));
    g_held = -1;
    g_acc  = 0.0f;
}

static void Put(int i, const glm::vec3& p, float s)
{
    Instance& in = g_Instances[i];
    in.pos   = p;
    in.scale = glm::vec3(s);
    in.yaw   = 0.0f;
    in.tilt  = glm::mat4(1.0f);
    in.vel   = glm::vec3(0.0f);
    in.body  = BodyState();
}

static float Len(const glm::vec3& v) { return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); }

static float MatDiff(const glm::mat4& a, const glm::mat4& b)
{
    float d = 0.0f;
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r)
            d = std::max(d, std::fabs(a[c][r] - b[c][r]));
    return d;
}

static glm::vec3 Center(int i)
{
    glm::vec3 a, b;
    GetMeshLocalBBox(g_Instances[i].mesh, &a, &b);
    return InstanceBodyCenter(g_Instances[i], a, b);
}

static bool IsStable(int i)
{
    glm::vec3 ax; float ang;
    return !NearestStablePose(g_Instances[i], 0.01f, &ax, &ang);
}

static bool Asleep(int i)
{
    const Instance& in = g_Instances[i];
    return in.body.motion == MOTION_REST && Len(in.vel) == 0.0f
        && Len(in.body.ang_vel) == 0.0f;
}

// ---------------------------------------------------------------------------
// j) Sanidade, conferida a cada quadro de todos os cenarios.
// ---------------------------------------------------------------------------
struct Sanity
{
    int   nan;         // quadros com NaN/inf em algum pegavel
    float out;         // maior distancia de um pegavel para fora da sala (m)
    float pen_rest;    // maior penetracao entre dois pegaveis parados (m)
    float pen_static;  // maior penetracao de um pegavel parado/rolando no cenario (m)
    std::string where; // descricao do pior caso
};
static Sanity S;
static Sanity S_all;

static void SanityReset() { S.nan = 0; S.out = 0; S.pen_rest = 0; S.pen_static = 0; S.where = ""; }

static bool FiniteV(const glm::vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

static float Penetration(const AABB& a, const AABB& b)
{
    float ox = std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x);
    float oy = std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y);
    float oz = std::min(a.max.z, b.max.z) - std::max(a.min.z, b.min.z);
    if (ox <= 0 || oy <= 0 || oz <= 0) return 0.0f;
    return std::min(ox, std::min(oy, oz));
}

// Distancia que a caixa passa para fora do volume interno da sala (ou do
// corredor da saida, que fica alem da parede +X acima do peitoril).
static float OutOfRoom(const AABB& b)
{
    float room = 0.0f;
    room = std::max(room, -Room::HALF_X - b.min.x);
    room = std::max(room,  b.max.x - Room::HALF_X);
    room = std::max(room, -b.min.y);
    room = std::max(room,  b.max.y - Room::HEIGHT);
    room = std::max(room, -Room::HALF_Z - b.min.z);
    room = std::max(room,  b.max.z - Room::HALF_Z);
    if (room <= 0.0f) return 0.0f;

    float hw = 0.5f*Room::EXIT_WIDTH;
    float exitv = 0.0f;
    exitv = std::max(exitv, -Room::HALF_X - b.min.x);
    exitv = std::max(exitv,  b.max.x - (Room::HALF_X + Room::THICKNESS + Puzzle::CORRIDOR_LENGTH));
    exitv = std::max(exitv,  Room::EXIT_Y0 - b.min.y);
    exitv = std::max(exitv,  b.max.y - Room::HEIGHT);
    exitv = std::max(exitv, -hw - b.min.z);
    exitv = std::max(exitv,  b.max.z - hw);
    return std::min(room, exitv);
}

static void CheckSanity()
{
    const size_t n = g_Instances.size();
    std::vector<AABB> bx(n);
    for (size_t i = 0; i < n; ++i)
        if (g_Instances[i].solid || g_Instances[i].grabbable) bx[i] = BoxOf((int)i);

    for (size_t i = 0; i < n; ++i)
    {
        const Instance& a = g_Instances[i];
        if (!a.grabbable || (int)i == g_held) continue;

        bool fin = FiniteV(a.pos) && FiniteV(a.vel) && FiniteV(a.body.ang_vel) && std::isfinite(a.yaw);
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                fin = fin && std::isfinite(a.tilt[c][r]);
        if (!fin) { ++S.nan; continue; }

        float o = OutOfRoom(bx[i]);
        if (o > S.out) { S.out = o; S.where = a.mesh + " fora da sala"; }

        const bool rest_a = a.body.motion == MOTION_REST && Len(a.vel) < 1e-6f;

        for (size_t j = 0; j < n; ++j)
        {
            if (j == i || !g_Instances[j].solid || (int)j == g_held) continue;
            const Instance& b = g_Instances[j];
            float p = Penetration(bx[i], bx[j]);
            if (p <= 0.0f) continue;

            if (!b.grabbable)
            {
                // Penetracao no cenario so conta para quem esta parado ou
                // rolando: durante o tombo (e logo depois de soltar da
                // aresta) a AABB do objeto girado cobre a quina do apoio sem
                // que o corpo de verdade entre nele.
                if (!rest_a && a.body.motion != MOTION_ROLL) continue;
                if (p > S.pen_static)
                {
                    S.pen_static = p;
                    char buf[160];
                    snprintf(buf, sizeof(buf), "%s[%d] dentro do cenario[%d] %.4f", a.mesh.c_str(), (int)i, (int)j, p);
                    S.where = buf;
                }
            }
            else if (j > i && rest_a && b.body.motion == MOTION_REST && Len(b.vel) < 1e-6f)
            {
                if (p > S.pen_rest)
                {
                    S.pen_rest = p;
                    char buf[160];
                    snprintf(buf, sizeof(buf), "%s[%d] x %s[%d] %.4f", a.mesh.c_str(), (int)i, b.mesh.c_str(), (int)j, p);
                    S.where = buf;
                }
            }
        }
    }
}

static bool SanityOk() { return S.nan == 0 && S.out <= 0.005f && S.pen_rest <= 0.005f && S.pen_static <= 0.005f; }

static void SanityMerge()
{
    S_all.nan += S.nan;
    if (S.out > S_all.out)               { S_all.out = S.out; }
    if (S.pen_rest > S_all.pen_rest)     { S_all.pen_rest = S.pen_rest; }
    if (S.pen_static > S_all.pen_static) { S_all.pen_static = S.pen_static; }
    if (!SanityOk() && S_all.where.empty()) S_all.where = S.where;
}

static std::string SanityStr()
{
    char buf[256];
    snprintf(buf, sizeof(buf), "sanidade: nan=%d fora=%.4f pen_repouso=%.4f pen_cenario=%.4f%s%s",
             S.nan, S.out, S.pen_rest, S.pen_static, SanityOk() ? "" : " pior: ", SanityOk() ? "" : S.where.c_str());
    return buf;
}

// ---------------------------------------------------------------------------
// Passo de quadro como no main.cpp: o delta time do quadro vai para um
// acumulador e a fisica dos objetos avanca em passos fixos FIXED_STEP (no
// maximo MAX_SUBSTEPS por quadro, o resto descartado). Jogador e
// LevelUpdate rodam por quadro, como la.
// ---------------------------------------------------------------------------
static void Frame(float fdt, bool move_player = false, PlayerInput in = NONE)
{
    if (move_player)
        UpdatePlayer(P, in, 0.0f, SolidBoxes(), fdt);

    AABB player_box = PlayerAABB(P);
    g_acc += fdt;
    int n = 0;
    while (g_acc >= FS && n < PhysicsConfig::MAX_SUBSTEPS)
    {
        UpdateFallingObjects(g_held, player_box, FS);
        g_acc -= FS;
        ++n;
    }
    if (g_acc >= FS)
        g_acc = 0.0f;

    LevelUpdate(L, g_held, PlayerAABB(P), fdt);
    CheckSanity();
}

// Roda passos fixos (um por "quadro") ate o objeto "i" dormir e ficar
// dormindo por "hold" segundos, ou ate tmax. Chama "each" a cada passo.
static float RunUntilAsleep(int i, float tmax, float hold,
                            std::function<void(float)> each = std::function<void(float)>(),
                            float fdt = FS)
{
    float t = 0.0f, asleep_for = 0.0f;
    while (t < tmax)
    {
        Frame(fdt);
        t += fdt;
        if (each) each(t);
        if (Asleep(i)) { asleep_for += fdt; if (asleep_for >= hold) break; }
        else asleep_for = 0.0f;
    }
    return t;
}

#if PHYSICS_ENABLE_TIPPING || PHYSICS_ENABLE_ROLLING
// ===========================================================================
// Cenarios com o M8 ligado.
// ===========================================================================

// a) Cubo magico 0.36 na borda +X de um dado 1.5 em (4.5, 0.75, 0).
struct EdgeResult
{
    glm::vec3 pos;
    glm::mat4 orient;
    float     yaw;
    float     t;
    unsigned  seen;      // bits (1 << MotionState) vistos
    bool      asleep;
    bool      stable;
};

static EdgeResult RunCubeEdge(float offset_out, float fdt, float tmax)
{
    Reset();
    SanityReset();
    int d0 = FindMesh("the_die"), r = FindMesh("the_rubik");
    Put(d0, glm::vec3(4.5f, 0.75f, 0.0f), 1.5f);
    const float edge = 4.5f + 0.75f;
    Put(r, glm::vec3(edge + offset_out, 1.5f + 0.18f + 0.002f, 0.0f), 0.36f);

    EdgeResult R;
    R.seen = 0;
    R.t = RunUntilAsleep(r, tmax, 0.5f, [&](float) { R.seen |= 1u << g_Instances[r].body.motion; }, fdt);
    R.pos    = Center(r);
    R.orient = InstanceOrientation(g_Instances[r]);
    R.yaw    = g_Instances[r].yaw;
    R.asleep = Asleep(r);
    R.stable = IsStable(r);
    return R;
}

static std::string SeenStr(unsigned seen)
{
    std::string s;
    for (int m = 0; m <= MOTION_ROLL; ++m)
        if (seen & (1u << m)) { if (!s.empty()) s += ","; s += MotionName(m); }
    return s;
}

static EdgeResult g_a_out;

static void ScenarioA()
{
    EdgeResult o = RunCubeEdge(0.036f, FS, 8.0f);
    g_a_out = o;
    bool ok = (o.seen & (1u << MOTION_TIP)) && o.asleep && o.stable
           && std::fabs(o.pos.y - 0.18f) < 0.003f && o.pos.x > 5.25f + 0.05f && o.pos.x < 6.0f - 0.18f + 0.002f
           && MatDiff(o.orient, glm::mat4(1.0f)) > 0.5f && SanityOk();
    Verdict("a1", ok, "cubo 3.6cm FORA da borda: estados {%s}, dormiu=%d em %.2fs, centro=(%.3f,%.3f,%.3f), face estavel=%d, girou=%d | %s",
            SeenStr(o.seen).c_str(), o.asleep, o.t, o.pos.x, o.pos.y, o.pos.z, o.stable,
            MatDiff(o.orient, glm::mat4(1.0f)) > 0.5f, SanityStr().c_str());
    SanityMerge();

    EdgeResult in = RunCubeEdge(-0.036f, FS, 4.0f);
    ok = in.asleep && !(in.seen & (1u << MOTION_TIP)) && std::fabs(in.pos.y - 1.68f) < 0.003f
      && std::fabs(in.pos.x - (5.25f - 0.036f)) < 0.001f && MatDiff(in.orient, glm::mat4(1.0f)) < 1e-4f && SanityOk();
    Verdict("a2", ok, "cubo 3.6cm DENTRO da borda: estados {%s}, centro=(%.3f,%.3f,%.3f) (esperado 5.214,1.680), sem girar=%d | %s",
            SeenStr(in.seen).c_str(), in.pos.x, in.pos.y, in.pos.z, MatDiff(in.orient, glm::mat4(1.0f)) < 1e-4f,
            SanityStr().c_str());
    SanityMerge();
}

// Direcao horizontal (angulo em XZ) do eixo de revolucao do peao.
static float AxisHeading(int i)
{
    glm::vec3 u = InstanceLocalAxisWorld(g_Instances[i], 1);
    return std::atan2(u.z, u.x);
}

static float WrapPi(float a)
{
    while (a >  3.14159265f) a -= 6.2831853f;
    while (a < -3.14159265f) a += 6.2831853f;
    return a;
}

// b) Peao na mesma borda.
//
// b1 (caso do usuario): peao EM PE com o centro 3,6 cm alem da borda +X. Ele
//    tomba como caixa sobre a aresta (eixo de giro = Z), cai e deita no piso.
//    Por simetria, todo o movimento fica no plano XY: o peao deita com o
//    proprio eixo ao longo de X e a unica velocidade que traz e ao longo
//    desse eixo (v = omega x r, com omega em Z). Rolar exige velocidade
//    PERPENDICULAR ao eixo (rola em torno dele), entao o certo aqui e ele
//    deitar e parar, sem rolar: um peao real so rola depois de tombar por
//    causa de assimetrias que este cenario ideal nao tem. O teste confere
//    que ele deita, para no piso e que o que sobrou de rolamento e ~0.
// b2 (o rolamento em arco): peao DEITADO no dado com o eixo ao longo de Z e
//    o centro 3,6 cm alem da borda +X. A borda agora e perpendicular ao
//    rolamento: ele sai rolando, cai, pousa rolando (RollOnLand), rola no
//    piso em ARCO (base mais larga que a cabeca: o eixo gira em torno do
//    vertical) e para pelo atrito de rolamento.
struct PawnRun
{
    unsigned  seen;
    bool      rolled_floor;   // houve ROLL depois de pousar no piso
    float     roll_dist;      // distancia rolada no piso (m)
    float     arc;            // quanto o eixo girou em torno do vertical rolando no piso (rad)
    float     t;
    glm::vec3 c;
};

static PawnRun RunPawn(int pw, float radius)
{
    PawnRun R;
    R.seen = 0; R.rolled_floor = false; R.roll_dist = 0.0f; R.arc = 0.0f;
    bool on_floor = false;
    float head_start = 0.0f, hmin = 0.0f, hmax = 0.0f;
    glm::vec3 last(0.0f);
    R.t = RunUntilAsleep(pw, 15.0f, 0.5f, [&](float) {
        const Instance& in = g_Instances[pw];
        R.seen |= 1u << in.body.motion;
        glm::vec3 c = Center(pw);
        if (!on_floor && c.y < radius + 0.01f && in.body.motion != MOTION_FALL) on_floor = true;
        if (on_floor && in.body.motion == MOTION_ROLL)
        {
            float h = AxisHeading(pw);
            if (!R.rolled_floor) { R.rolled_floor = true; last = c; head_start = h; hmin = hmax = 0.0f; }
            float dh = WrapPi(h - head_start);
            hmin = std::min(hmin, dh); hmax = std::max(hmax, dh);
            R.roll_dist += std::sqrt((c.x-last.x)*(c.x-last.x) + (c.z-last.z)*(c.z-last.z));
            last = c;
        }
    });
    R.arc = hmax - hmin;
    R.c = Center(pw);
    return R;
}

static void ScenarioB()
{
    const float s = 0.56f, half_h = 0.5f*s, radius = 0.335f*s;

    // b1
    {
        Reset();
        SanityReset();
        int d0 = FindMesh("the_die"), pw = FindMesh("the_pawn");
        Put(d0, glm::vec3(4.5f, 0.75f, 0.0f), 1.5f);
        Put(pw, glm::vec3(5.25f + 0.036f, 1.5f + half_h + 0.002f, 0.0f), s);
        PawnRun R = RunPawn(pw, radius);
        glm::vec3 u = InstanceLocalAxisWorld(g_Instances[pw], 1);
        bool lying = PawnIsLying(g_Instances[pw]);
        bool ok = (R.seen & (1u << MOTION_TIP)) && lying && Asleep(pw) && IsStable(pw)
               && std::fabs(R.c.y - radius) < 0.004f && R.c.x > 5.25f && std::fabs(u.x) > 0.95f
               && R.roll_dist < 0.02f && SanityOk();
        Verdict("b1", ok, "peao EM PE 3.6cm fora: estados {%s}, deitado=%d com eixo=(%.2f,%.2f,%.2f) (ao longo de X, sem velocidade perpendicular: nao rola, esperado), rolou no piso %.3f m, dormiu=%d em %.2fs, centro=(%.3f,%.3f,%.3f) y esperado %.3f | %s",
                SeenStr(R.seen).c_str(), lying, u.x, u.y, u.z, R.roll_dist, Asleep(pw), R.t, R.c.x, R.c.y, R.c.z, radius,
                SanityStr().c_str());
        SanityMerge();
    }

    // b2
    {
        Reset();
        SanityReset();
        int d0 = FindMesh("the_die"), pw = FindMesh("the_pawn");
        Put(d0, glm::vec3(4.5f, 0.75f, 0.0f), 1.5f);
        Put(pw, glm::vec3(0.0f), s);
        g_Instances[pw].tilt = RotationAboutAxis(glm::vec3(1.0f, 0.0f, 0.0f), 1.5707963f);   // Y local -> Z de mundo
        glm::vec3 la, lb;
        GetMeshLocalBBox("the_pawn", &la, &lb);
        SetInstanceBodyCenter(g_Instances[pw], la, lb, glm::vec3(5.25f + 0.036f, 1.5f + radius + 0.002f, 0.0f));
        PawnRun R = RunPawn(pw, radius);
        bool lying = PawnIsLying(g_Instances[pw]);
        bool ok = (R.seen & (1u << MOTION_ROLL)) && (R.seen & (1u << MOTION_FALL)) && R.rolled_floor
               && R.roll_dist > 0.05f && R.arc > 0.05f && lying && Asleep(pw)
               && std::fabs(R.c.y - radius) < 0.004f && SanityOk();
        Verdict("b2", ok, "peao DEITADO (eixo em Z) 3.6cm fora: estados {%s}, rolou no piso=%d %.3f m, eixo girou %.1f graus (arco), deitado=%d, dormiu=%d em %.2fs, centro=(%.3f,%.3f,%.3f) | %s",
                SeenStr(R.seen).c_str(), R.rolled_floor, R.roll_dist, R.arc*57.2958f, lying, Asleep(pw), R.t,
                R.c.x, R.c.y, R.c.z, SanityStr().c_str());
        SanityMerge();
    }
}

// c) Bola: c1 com o centro alem da borda +X do dado grande (rola para fora,
//    cai, bate na parede +X, rola no piso, para); c2 lancada no piso contra a
//    parede +X (rola, quica, volta, para).
static void ScenarioC()
{
    Reset();
    int ball = FindMesh("the_sphere");
    if (ball < 0 || g_Instances[ball].shape != SHAPE_SPHERE)
    {
        Verdict("c", false, "nao achei a bola (the_sphere pegavel com SHAPE_SPHERE) na cena");
        return;
    }
    const float rr = g_Instances[ball].scale.x;   // malha com bbox +-1: raio = escala

    // c1
    {
        Reset();
        SanityReset();
        int d0 = FindMesh("the_die");
        Put(d0, glm::vec3(4.5f, 0.75f, 0.0f), 1.5f);
        Put(ball, glm::vec3(5.25f + 0.02f, 1.5f + rr + 0.002f, 0.0f), rr);
        g_Instances[ball].shape = SHAPE_SPHERE;

        unsigned seen = 0, seen_after_land = 0;
        bool fell = false, landed = false, bounced = false;
        float vmax = 0.0f, xmax = -1e9f;
        float t = RunUntilAsleep(ball, 30.0f, 0.5f, [&](float) {
            const Instance& in = g_Instances[ball];
            seen |= 1u << in.body.motion;
            if (in.body.motion == MOTION_FALL) fell = true;
            if (fell && in.body.motion != MOTION_FALL && Center(ball).y < rr + 0.01f) landed = true;
            if (landed) seen_after_land |= 1u << in.body.motion;
            vmax = std::max(vmax, in.vel.x);
            if (vmax > 0.1f && in.vel.x < -0.01f) bounced = true;
            xmax = std::max(xmax, BoxOf(ball).max.x);
        });
        glm::vec3 c = Center(ball);
        bool ok = (seen & (1u << MOTION_ROLL)) && fell && landed && (seen_after_land & (1u << MOTION_ROLL))
               && bounced && xmax <= Room::HALF_X + 0.002f && Asleep(ball) && std::fabs(c.y - rr) < 0.003f && SanityOk();
        Verdict("c1", ok, "bola alem da borda: estados {%s}, caiu=%d, rolou no piso=%d, quicou=%d (vx max %.2f), x_max=%.4f (parede %.3f), dormiu=%d em %.2fs, centro=(%.3f,%.3f,%.3f) | %s",
                SeenStr(seen).c_str(), fell, (seen_after_land & (1u << MOTION_ROLL)) != 0, bounced, vmax, xmax, Room::HALF_X,
                Asleep(ball), t, c.x, c.y, c.z, SanityStr().c_str());
        SanityMerge();
    }

    // c2
    {
        Reset();
        SanityReset();
        Put(ball, glm::vec3(4.2f, rr + 0.005f, -3.0f), rr);
        g_Instances[ball].shape = SHAPE_SPHERE;
        g_Instances[ball].vel = glm::vec3(2.0f, 0.0f, 0.0f);   // como solto da mao a 2 m/s
        g_Instances[ball].body.motion = MOTION_FALL;

        unsigned seen = 0;
        bool bounced = false;
        float xmax = -1e9f, vin = 0.0f, vout = 0.0f;
        float t = RunUntilAsleep(ball, 30.0f, 0.5f, [&](float) {
            const Instance& in = g_Instances[ball];
            seen |= 1u << in.body.motion;
            if (!bounced) vin = std::max(vin, in.vel.x);
            if (in.vel.x < -0.01f) { bounced = true; vout = std::min(vout, in.vel.x); }
            xmax = std::max(xmax, BoxOf(ball).max.x);
        });
        glm::vec3 c = Center(ball);
        bool ok = (seen & (1u << MOTION_ROLL)) && bounced && xmax <= Room::HALF_X + 0.002f
               && Asleep(ball) && std::fabs(c.y - rr) < 0.003f && c.x < Room::HALF_X - rr - 0.01f && SanityOk();
        Verdict("c2", ok, "bola lancada a 2 m/s contra a parede +X: estados {%s}, quicou=%d (vx %.2f -> %.2f), x_max=%.4f (parede %.3f), dormiu=%d em %.2fs, centro=(%.3f,%.3f,%.3f) | %s",
                SeenStr(seen).c_str(), bounced, vin, vout, xmax, Room::HALF_X, Asleep(ball), t, c.x, c.y, c.z,
                SanityStr().c_str());
        SanityMerge();
    }
}

// d) Cena inicial parada por 10 s (em quadros de 1/60 s).
static void ScenarioD()
{
    Reset();
    SanityReset();
    std::vector<Instance> before = g_Instances;
    for (int f = 0; f < 600; ++f) Frame(1.0f/60.0f);

    float drift = 0.0f, rot = 0.0f;
    std::string worst = "-";
    bool all_rest = true;
    for (size_t i = 0; i < g_Instances.size(); ++i)
    {
        if (!g_Instances[i].grabbable) continue;
        float d = Len(g_Instances[i].pos - before[i].pos);
        float a = std::max(MatDiff(g_Instances[i].tilt, before[i].tilt), std::fabs(g_Instances[i].yaw - before[i].yaw));
        if (d > drift) { drift = d; worst = g_Instances[i].mesh; }
        rot = std::max(rot, a);
        if (!Asleep((int)i)) all_rest = false;
    }
    bool ok = drift < 0.001f && rot < 1e-5f && all_rest && SanityOk();
    Verdict("d", ok, "cena inicial 10 s parada: deriva max %.6f m (%s), rotacao max %.2e, todos dormindo=%d | %s",
            drift, worst.c_str(), rot, all_rest, SanityStr().c_str());
    SanityMerge();
}

// e) Pilha de dois cubos caindo junta.
static void ScenarioE()
{
    Reset();
    SanityReset();
    int d0 = FindMesh("the_die", 0), d1 = FindMesh("the_die", 1);
    Put(d0, glm::vec3(2.0f, 1.5f, -3.0f), 0.4f);
    Put(d1, glm::vec3(2.0f, 1.9f, -3.0f), 0.4f);
    float gap = 0.0f;
    float t = RunUntilAsleep(d1, 6.0f, 0.5f, [&](float) {
        gap = std::max(gap, std::fabs((g_Instances[d1].pos.y - 0.2f) - (g_Instances[d0].pos.y + 0.2f)));
    });
    glm::vec3 a = g_Instances[d0].pos, b = g_Instances[d1].pos;
    float rot = std::max(MatDiff(g_Instances[d0].tilt, glm::mat4(1.0f)), MatDiff(g_Instances[d1].tilt, glm::mat4(1.0f)));
    bool ok = gap < 0.002f && Asleep(d0) && Asleep(d1) && std::fabs(a.y - 0.2f) < 0.002f && std::fabs(b.y - 0.6f) < 0.002f
           && std::fabs(a.x - 2.0f) + std::fabs(a.z + 3.0f) + std::fabs(b.x - 2.0f) + std::fabs(b.z + 3.0f) < 0.002f
           && rot < 1e-4f && SanityOk();
    Verdict("e", ok, "pilha caindo junta: folga max %.4f m, dormiram=%d,%d em %.2fs, y=%.3f/%.3f (esperado 0.200/0.600), rotacao %.1e | %s",
            gap, Asleep(d0), Asleep(d1), t, a.y, b.y, rot, SanityStr().c_str());
    SanityMerge();
}

// f) Pego no meio do tombo. f1: levado para o ar e solto (cai e assenta no
//    piso). f2: solto torto ja encostado no piso (assenta numa face).
static void ScenarioF()
{
    for (int variant = 1; variant <= 2; ++variant)
    {
        Reset();
        SanityReset();
        int d0 = FindMesh("the_die"), r = FindMesh("the_rubik");
        Put(d0, glm::vec3(4.5f, 0.75f, 0.0f), 1.5f);
        Put(r, glm::vec3(5.25f + 0.036f, 1.5f + 0.18f + 0.002f, 0.0f), 0.36f);

        // Ate estar no meio do tombo.
        float t = 0.0f, ang = 0.0f;
        bool tipping = false;
        while (t < 3.0f)
        {
            Frame(FS); t += FS;
            if (g_Instances[r].body.motion == MOTION_TIP) tipping = true;
            ang = std::acos(std::max(-1.0f, std::min(1.0f, InstanceLocalAxisWorld(g_Instances[r], 1).y)));
            if (tipping && ang > 0.25f) break;
        }
        bool was_tipping = g_Instances[r].body.motion == MOTION_TIP;

        // Pega: grab.cpp torna o objeto nao solido e o move com a camera.
        g_held = r;
        g_Instances[r].solid = false;
        glm::mat4 tilt_held = g_Instances[r].tilt;
        bool body_zero = true, frozen = true;
        glm::vec3 hand = (variant == 1) ? glm::vec3(3.0f, 1.2f, 2.5f) : glm::vec3(3.0f, 0.5f, 2.5f);
        g_Instances[r].pos = hand;
        for (int k = 0; k < 120; ++k)
        {
            Frame(FS);
            const Instance& in = g_Instances[r];
            body_zero = body_zero && in.body.motion == MOTION_REST && Len(in.body.ang_vel) == 0.0f
                     && in.body.angle == 0.0f && Len(in.vel) == 0.0f;
            frozen = frozen && Len(in.pos - hand) == 0.0f && MatDiff(in.tilt, tilt_held) == 0.0f;
        }

        if (variant == 2)
        {
            // Solta com a base torta encostando 1 mm acima do piso.
            AABB b = BoxOf(r);
            g_Instances[r].pos.y -= b.min.y - 0.001f;
        }
        g_held = -1;
        g_Instances[r].solid = true;

        unsigned seen = 0;
        float t2 = RunUntilAsleep(r, 6.0f, 0.5f, [&](float) { seen |= 1u << g_Instances[r].body.motion; });
        glm::vec3 c = Center(r);
        bool ok = was_tipping && body_zero && frozen && Asleep(r) && IsStable(r) && std::fabs(c.y - 0.18f) < 0.003f
               && std::fabs(c.x - hand.x) < 0.3f && std::fabs(c.z - hand.z) < 0.3f && SanityOk();
        char id[8]; snprintf(id, sizeof(id), "f%d", variant);
        Verdict(id, ok, "%s: pego no TIP=%d (%.0f graus), na mao body zerado=%d parado=%d; solto: estados {%s}, dormiu=%d em %.2fs, face estavel=%d, centro=(%.3f,%.3f,%.3f) | %s",
                variant == 1 ? "pego no meio do tombo e solto no ar" : "pego no meio do tombo e solto torto encostado no piso",
                was_tipping, ang*57.2958f, body_zero, frozen, SeenStr(seen).c_str(), Asleep(r), t2, IsStable(r),
                c.x, c.y, c.z, SanityStr().c_str());
        SanityMerge();
    }
}

// h) Puzzle completo do t_base item 7, com o passo fixo e o M8 ligado.
static void ScenarioH()
{
    Reset();
    SanityReset();
    PlayerInit(P, glm::vec3(2.3f, 0.0f, 0.0f));
    int brick = FindMesh("the_cube"), d0 = FindMesh("the_die"), r = FindMesh("the_rubik");
    Put(brick, glm::vec3(-5.0f, 0.18f, -3.0f), 0.36f);
    Put(d0, glm::vec3(4.5f, 3.0f, 0.0f), 1.5f);
    Put(r, glm::vec3(3.25f, 2.5f, 0.0f), 0.85f);
    const float dt = 1.0f/120.0f;
    for (int k = 0; k < 400; ++k) Frame(dt, true, NONE);

    PlayerInput fw = {1,0,0,0,0}, fj = {1,0,0,0,1}, ju = {0,0,0,0,1};
    int phase = 0;
    float levels[2] = {0.85f, 1.5f};
    for (int k = 0; k < 3000 && !L.won; ++k)
    {
        PlayerInput in = NONE;
        if (phase < 2)
        {
            if (P.on_ground && P.pos.y > levels[phase] - 0.01f) phase++;
            if (phase < 2) { if (P.on_ground) in = ju; else if (P.pos.y > levels[phase] + 0.02f) in = fw; }
        }
        if (phase == 2) in = (P.pos.x > 4.95f && P.on_ground) ? fj : fw;
        Frame(dt, true, in);
    }
    bool ok = L.won && SanityOk();
    Verdict("h", ok, "puzzle: porta=%.2f venceu=%d fase=%d P=(%.2f,%.2f,%.2f) | rubik=(%.2f,%.2f,%.2f) %s dado=(%.2f,%.2f,%.2f) %s | %s",
            L.door_offset, L.won, phase, P.pos.x, P.pos.y, P.pos.z,
            g_Instances[r].pos.x, g_Instances[r].pos.y, g_Instances[r].pos.z, MotionName(g_Instances[r].body.motion),
            g_Instances[d0].pos.x, g_Instances[d0].pos.y, g_Instances[d0].pos.z, MotionName(g_Instances[d0].body.motion),
            SanityStr().c_str());
    SanityMerge();
}

// i) Independencia de FPS: o cenario a1 com quadros de 1/30, 1/60 e 1/144 s.
static void ScenarioI()
{
    const float fdts[3] = { 1.0f/30.0f, 1.0f/60.0f, 1.0f/144.0f };
    float dpos = 0.0f, dori = 0.0f;
    bool all_ok = true;
    char detail[512] = "";
    for (int q = 0; q < 3; ++q)
    {
        EdgeResult e = RunCubeEdge(0.036f, fdts[q], 8.0f);
        all_ok = all_ok && e.asleep && e.stable && SanityOk();
        SanityMerge();
        float dp = Len(e.pos - g_a_out.pos);
        float dr = MatDiff(e.orient, g_a_out.orient);
        dpos = std::max(dpos, dp);
        dori = std::max(dori, dr);
        char buf[160];
        snprintf(buf, sizeof(buf), "%s1/%.0f: (%.4f,%.4f,%.4f) %.2fs", q ? "; " : "", 1.0f/fdts[q], e.pos.x, e.pos.y, e.pos.z, e.t);
        strncat(detail, buf, sizeof(detail) - strlen(detail) - 1);
    }
    bool ok = all_ok && dpos < 0.01f && dori < 0.01f;
    Verdict("i", ok, "FPS: %s | maior diferenca para o passo-a-passo: posicao %.5f m, orientacao %.5f", detail, dpos, dori);
}

// j) LevelRestart zera body/tilt, e resumo da sanidade de todos os cenarios.
static void ScenarioJ()
{
    // Deixa varios objetos tortos/em movimento e reinicia.
    RunCubeEdge(0.036f, FS, 0.3f);   // cubo no meio do tombo
    int ball = FindMesh("the_sphere");
    if (ball >= 0) { g_Instances[ball].vel = glm::vec3(1.0f, 0.0f, 0.0f); g_Instances[ball].body.motion = MOTION_ROLL; }
    for (int k = 0; k < 30; ++k) Frame(FS);

    LevelRestart(L);
    std::vector<Instance> fresh;
    BuildScene(fresh);
    bool clean = true;
    for (size_t i = 0; i < g_Instances.size(); ++i)
    {
        const Instance& in = g_Instances[i];
        if (!in.grabbable) continue;
        clean = clean && MatDiff(in.tilt, glm::mat4(1.0f)) == 0.0f && in.body.motion == MOTION_REST
             && Len(in.body.ang_vel) == 0.0f && in.body.angle == 0.0f && in.body.timer == 0.0f
             && Len(in.vel) == 0.0f && Len(in.pos - fresh[i].pos) == 0.0f;
    }
    bool ok = clean && S_all.nan == 0 && S_all.out <= 0.005f && S_all.pen_rest <= 0.005f && S_all.pen_static <= 0.005f;
    Verdict("j", ok, "LevelRestart zera body/tilt/vel=%d; sanidade em todos: nan=%d fora=%.4f pen_repouso=%.4f pen_cenario=%.4f (limite 0.005)%s%s",
            clean, S_all.nan, S_all.out, S_all.pen_rest, S_all.pen_static,
            S_all.where.empty() ? "" : " pior: ", S_all.where.c_str());
}

int main()
{
    printf("== t_m8: M8 ligado (TIPPING=%d ROLLING=%d), passo fixo %.5f s\n",
           PhysicsConfig::ENABLE_TIPPING, PhysicsConfig::ENABLE_ROLLING, FS);
    S_all.nan = 0; S_all.out = S_all.pen_rest = S_all.pen_static = 0;
    ScenarioA();
    ScenarioB();
    ScenarioC();
    ScenarioD();
    ScenarioE();
    ScenarioF();
    ScenarioH();
    ScenarioI();
    ScenarioJ();
    printf("== t_m8: %d cenario(s) FALHOU\n", g_fails);
    return g_fails;
}

#else
// ===========================================================================
// g) Flags desligadas: os itens do t_base (passo de 1/120 s direto, sem
//    acumulador, como no M7) devem dar os numeros do M7.
// ===========================================================================
static void StepBase(int n, PlayerInput in = NONE)
{
    const float dt = 1.0f/120.0f;
    for (int k = 0; k < n; ++k)
    {
        UpdatePlayer(P, in, 0.0f, SolidBoxes(), dt);
        UpdateFallingObjects(-1, PlayerAABB(P), dt);
        LevelUpdate(L, -1, PlayerAABB(P), dt);
        CheckSanity();
    }
}

static void PutBase(int i, glm::vec3 p, float s)
{
    g_Instances[i].pos = p; g_Instances[i].scale = glm::vec3(s); g_Instances[i].yaw = 0; g_Instances[i].vel = glm::vec3(0);
}

int main()
{
    printf("== t_m8: flags do M8 DESLIGADAS (TIPPING=%d ROLLING=%d): cenario g, comparacao com o M7\n",
           PhysicsConfig::ENABLE_TIPPING, PhysicsConfig::ENABLE_ROLLING);
    SanityReset();
    BuildScene(g_Instances); LevelInit(L); PlayerInit(P, glm::vec3(-4.6f, 0, 0));
    const float dt = 1.0f/120.0f;
    int d0 = FindMesh("the_die"), d1 = FindMesh("the_die", 1), r = FindMesh("the_rubik"), pw = FindMesh("the_pawn");
    StepBase(240);

    // item 2
    PutBase(d0, glm::vec3(4.5f, 0.75f, 0), 1.5f); StepBase(60);
    const float edge = 5.25f;
    PutBase(r, glm::vec3(edge + 0.036f, 1.5f + 0.18f + 0.3f, 0), 0.36f);
    float t = 0;
    for (int k = 0; k < 600; ++k) { StepBase(1); t += dt; if (g_Instances[r].pos.y < 0.19f && std::fabs(g_Instances[r].vel.y) < 1e-6) break; }
    glm::vec3 p2 = g_Instances[r].pos;
    bool ok2 = std::fabs(t - 0.98f) < 0.015f && std::fabs(p2.x - 5.82f) < 0.006f && std::fabs(p2.y - 0.18f) < 0.001f;
    // item 3
    PutBase(r, glm::vec3(edge - 0.036f, 1.5f + 0.18f + 0.3f, 0), 0.36f); StepBase(600);
    glm::vec3 p3 = g_Instances[r].pos;
    bool ok3 = std::fabs(p3.y - 1.68f) < 0.001f && std::fabs(p3.x - 5.214f) < 0.001f;
    // item 4 (o t_base poe o cubo magico 0 em cima do cubo magico 1 da cena,
    // em (-1.2, 0.16, 2.1); aqui ele sai da frente para a checagem de
    // sobreposicao valer, sem mudar a ponte)
    { int r1 = FindMesh("the_rubik", 1); if (r1 >= 0) g_Instances[r1].pos = glm::vec3(-0.2f, 0.16f, 3.6f); }
    PutBase(d1, glm::vec3(-2, 0.17f, 2), 0.34f); PutBase(r, glm::vec3(-2 + 0.6f, 0.17f, 2), 0.34f); StepBase(30);
    PutBase(pw, glm::vec3(-2 + 0.3f, 0.34f + 0.28f + 0.2f, 2), 0.56f); StepBase(300);
    glm::vec3 p4 = g_Instances[pw].pos;
    bool ok4 = std::fabs(p4.y - 0.62f) < 0.001f && std::fabs(p4.x + 1.7f) < 0.001f;
    // item 5
    PutBase(d0, glm::vec3(2, 1.5f, -3), 0.4f); PutBase(d1, glm::vec3(2, 1.9f, -3), 0.4f);
    float gap = 0;
    for (int k = 0; k < 120; ++k) { StepBase(1); gap = std::max(gap, std::fabs((g_Instances[d1].pos.y - 0.2f) - (g_Instances[d0].pos.y + 0.2f))); }
    bool ok5 = gap < 1e-4f;
    // item 6
    PutBase(d0, glm::vec3(5.0f, 0.5f, -3.78f), 1.0f); StepBase(30);
    PutBase(r, glm::vec3(5.0f, 1.0f + 0.18f + 0.1f, -4.32f), 0.36f); StepBase(300);
    glm::vec3 p6 = g_Instances[r].pos;
    bool ok6 = std::fabs((p6.z - 0.18f) + 4.5f) < 0.0005f && std::fabs(p6.y - 1.18f) < 0.001f;
    // nada tombou nem girou
    bool untouched = true;
    for (size_t i = 0; i < g_Instances.size(); ++i)
        if (g_Instances[i].grabbable)
            untouched = untouched && MatDiff(g_Instances[i].tilt, glm::mat4(1.0f)) == 0.0f
                     && Len(g_Instances[i].body.ang_vel) == 0.0f
                     && g_Instances[i].body.motion != MOTION_TIP && g_Instances[i].body.motion != MOTION_SETTLE
                     && g_Instances[i].body.motion != MOTION_ROLL;

    bool ok = ok2 && ok3 && ok4 && ok5 && ok6 && untouched && SanityOk();
    Verdict("g", ok, "flags desligadas = M7: item2 %.2fs (%.2f,%.3f) [M7 0.98s (5.82,0.180)] %s; item3 y=%.3f x=%.3f [1.680 5.214] %s; item4 y=%.3f x=%.3f [0.620 -1.700] %s; item5 folga %.4f %s; item6 z_min=%.3f y=%.3f [-4.500 1.180] %s; sem tilt/ang_vel/TIP/ROLL=%d | %s",
            t, p2.x, p2.y, ok2 ? "ok" : "DIFERE", p3.y, p3.x, ok3 ? "ok" : "DIFERE", p4.y, p4.x, ok4 ? "ok" : "DIFERE",
            gap, ok5 ? "ok" : "DIFERE", p6.z - 0.18f, p6.y, ok6 ? "ok" : "DIFERE", untouched, SanityStr().c_str());
    printf("== t_m8 (flags desligadas): %d cenario(s) FALHOU\n", g_fails);
    return g_fails;
}
#endif
