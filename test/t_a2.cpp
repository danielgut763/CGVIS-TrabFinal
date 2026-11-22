// Testes do tombo (a2-tip, M8): TipBegin/TipStep/AirSpinStep/SettleBegin/
// SettleStep, sem janela. Ver tools/check.sh.
//
// Para nao depender de physics.cpp (a4-integ), este teste tem um "driver"
// proprio que segue o protocolo combinado com a4 (queda vertical e apoio
// como no M7, depois as funcoes de tip.h conforme o estado). Com
// TEST_USE_PHYSICS=1 no ambiente, usa UpdateFallingObjects de physics.cpp.
//
// Medida de "atravessar": profundidade maxima de interpenetracao entre o
// CORPO de verdade (caixa orientada; cilindro para o peao) e as caixas
// solidas, por amostragem da superficie do corpo (pontos dentro das caixas)
// e das arestas das caixas (pontos dentro do corpo). Pega tambem a aresta
// do apoio entrando na face do cubo que tomba.
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include "scene.h"
#include "physics.h"
#include "rigid.h"
#include "tip.h"
#include "roll.h"
#include "grab.h"

// ---------------------------------------------------------------------------
// Stubs fracos de roll.cpp (a3-roll): valem so enquanto roll.cpp nao existe.
// ---------------------------------------------------------------------------
__attribute__((weak)) bool RollsNow(const Instance& inst)
{ return inst.shape == SHAPE_SPHERE || (inst.shape == SHAPE_PAWN && PawnIsLying(inst)); }
__attribute__((weak)) bool RollHandlesOverhang(const Instance& inst, const glm::vec3&)
{ return inst.shape == SHAPE_SPHERE; }
__attribute__((weak)) void RollOnLand(Instance& inst, PhysicsContext&)
{ inst.body.motion = MOTION_REST; }
__attribute__((weak)) void RollStep(Instance& inst, PhysicsContext&, const SupportInfo&, float)
{ inst.vel.x = inst.vel.z = 0; inst.body.motion = MOTION_REST; }
__attribute__((weak)) void RollAfterMove(Instance&, PhysicsContext&, const glm::vec3&, const bool[3], float) {}

static int g_fail = 0;
#define CHECK(cond, ...) do { if (cond) { printf("  ok   "); } else { printf("  FALHA "); ++g_fail; } printf(__VA_ARGS__); printf("\n"); } while (0)

static float D3(const glm::vec3& a, const glm::vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static bool Finite(const glm::vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
static bool FiniteInst(const Instance& in)
{
    if (!Finite(in.pos) || !Finite(in.vel) || !Finite(in.body.ang_vel) || !std::isfinite(in.yaw)) return false;
    for (int c = 0; c < 4; ++c) for (int r = 0; r < 4; ++r) if (!std::isfinite(in.tilt[c][r])) return false;
    return true;
}

static AABB g_player = { glm::vec3(-5.9f, 0.0f, 4.0f), glm::vec3(-5.5f, 1.8f, 4.4f) };

static void BuildBoxes(std::vector<AABB>& boxes, std::vector<int>& owner)
{
    boxes.clear(); owner.clear();
    for (size_t i = 0; i < g_Instances.size(); ++i)
    {
        const Instance& in = g_Instances[i];
        if (!in.solid) continue;
        glm::vec3 a, b; GetMeshLocalBBox(in.mesh, &a, &b);
        boxes.push_back(InstanceCollisionBox((int)i, a, b));
        owner.push_back((int)i);
    }
    boxes.push_back(g_player);
    owner.push_back(-1);
}

// ---------------------------------------------------------------------------
// Driver com o protocolo de a4-integ (mensagem de 14:01).
// ---------------------------------------------------------------------------
static void DriverStep(float dt)
{
    std::vector<AABB> boxes; std::vector<int> owner;
    BuildBoxes(boxes, owner);

    std::vector<size_t> order;
    for (size_t k = 0; k < boxes.size(); ++k)
        if (owner[k] >= 0 && g_Instances[owner[k]].grabbable) order.push_back(k);
    std::sort(order.begin(), order.end(), [&boxes](size_t a, size_t b) { return boxes[a].min.y < boxes[b].min.y; });

    for (size_t m = 0; m < order.size(); ++m)
    {
        int k = (int)order[m], idx = owner[k];
        Instance& inst = g_Instances[idx];
        PhysicsContext ctx;
        ctx.boxes = &boxes; ctx.owner = &owner; ctx.self = k; ctx.instance = idx;
        GetMeshLocalBBox(inst.mesh, &ctx.lmin, &ctx.lmax);
        boxes[k] = InstanceCollisionBox(inst, idx, ctx.lmin, ctx.lmax);

        // TIP: so TipStep (ele mesmo percebe se o apoio sumiu).
        if (inst.body.motion == MOTION_TIP) { TipStep(inst, ctx, dt); continue; }
        if (inst.body.motion == MOTION_SETTLE) { SettleStep(inst, ctx, dt); continue; }

        AABB& box = boxes[k];
        float prev_vy = inst.vel.y;
        float v = inst.vel.y - PhysicsConfig::GRAVITY * dt;
        if (v < -PhysicsConfig::MAX_FALL_SPEED) v = -PhysicsConfig::MAX_FALL_SPEED;
        float dy = v * dt, support = 0; int support_k = -1; bool supported = false;
        if (FindSupportHeight(box, boxes, k, PhysicsConfig::SUPPORT_TOL, &support, &support_k)
            && box.min.y + dy <= support)
        { supported = true; dy = support - box.min.y; v = 0.0f; }
        inst.pos.y += dy; box.min.y += dy; box.max.y += dy; inst.vel.y = v;

        bool was_fall = (inst.body.motion == MOTION_FALL) || prev_vy < 0.0f;
        if (!supported)
        {
            inst.body.motion = MOTION_FALL;
            AirSpinStep(inst, ctx, dt);
        }
        else if (was_fall)
        {
            inst.vel.x = inst.vel.z = 0.0f;
            if (!SettleBegin(inst, ctx)) { inst.body.motion = MOTION_REST; inst.body.ang_vel = glm::vec3(0.0f); }
            continue;
        }

        if (supported)
        {
            SupportInfo sup; sup.supported = true; sup.height = support; sup.box_index = support_k;
            sup.unbalanced = false; sup.overhang = glm::vec3(0.0f);
            if (SupportFootprint(box, boxes, k, support, PhysicsConfig::SUPPORT_TOL, &sup.fx0, &sup.fx1, &sup.fz0, &sup.fz1))
            {
                glm::vec3 c = InstanceBodyCenter(inst, ctx.lmin, ctx.lmax);
                const float mg = PhysicsConfig::BALANCE_MARGIN;
                float ox = 0, oz = 0;
                if (c.x < sup.fx0 - mg) ox = c.x - sup.fx0; else if (c.x > sup.fx1 + mg) ox = c.x - sup.fx1;
                if (c.z < sup.fz0 - mg) oz = c.z - sup.fz0; else if (c.z > sup.fz1 + mg) oz = c.z - sup.fz1;
                sup.overhang = glm::vec3(ox, 0, oz); sup.unbalanced = (ox != 0 || oz != 0);
            }
            if (sup.unbalanced)
            {
                if (TipBegin(inst, ctx, sup)) continue;
                float len = std::sqrt(sup.overhang.x*sup.overhang.x + sup.overhang.z*sup.overhang.z);
                inst.vel.x += sup.overhang.x/len * PhysicsConfig::SLIDE_ACCEL * dt;
                inst.vel.z += sup.overhang.z/len * PhysicsConfig::SLIDE_ACCEL * dt;
                float sp = std::sqrt(inst.vel.x*inst.vel.x + inst.vel.z*inst.vel.z);
                if (sp > PhysicsConfig::SLIDE_MAX_SPEED) { inst.vel.x *= PhysicsConfig::SLIDE_MAX_SPEED/sp; inst.vel.z *= PhysicsConfig::SLIDE_MAX_SPEED/sp; }
            }
            else { inst.vel.x = inst.vel.z = 0.0f; if (inst.body.motion == MOTION_ROLL) inst.body.motion = MOTION_REST; }
        }

        if (inst.vel.x != 0.0f || inst.vel.z != 0.0f)
        {
            glm::vec3 delta(inst.vel.x * dt, 0.0f, inst.vel.z * dt);
            bool hit[3] = { false, false, false };
            glm::vec3 ap = ResolveAABBMovement(box, delta, boxes, k, hit);
            inst.pos.x += ap.x; inst.pos.z += ap.z;
            box.min.x += ap.x; box.max.x += ap.x; box.min.z += ap.z; box.max.z += ap.z;
            if (hit[0]) inst.vel.x = 0.0f;
            if (hit[2]) inst.vel.z = 0.0f;
        }
    }
}

static bool g_use_physics = false;
static void Step(float dt)
{
    if (g_use_physics) UpdateFallingObjects(-1, g_player, dt);
    else DriverStep(dt);
}

// ---------------------------------------------------------------------------
// Profundidade de interpenetracao corpo x caixas, por amostragem.
// ---------------------------------------------------------------------------
struct Geom { glm::vec3 c, e[3]; float h[3]; int shape; };
static Geom MakeGeom(const Instance& in)
{
    Geom g; glm::vec3 a, b; GetMeshLocalBBox(in.mesh, &a, &b);
    glm::mat4 O = InstanceOrientation(in);
    for (int i = 0; i < 3; ++i)
    {
        g.e[i] = glm::vec3(O[i][0], O[i][1], O[i][2]);
        float l = std::sqrt(D3(g.e[i], g.e[i])); g.e[i] = g.e[i] * (1.0f/l);
        g.h[i] = 0.5f*(b[i]-a[i])*in.scale[i];
    }
    g.c = InstanceBodyCenter(in, a, b); g.shape = in.shape;
    return g;
}
static float DepthInBody(const Geom& g, const glm::vec3& p)
{
    glm::vec3 d = p - g.c; float q[3] = { D3(d, g.e[0]), D3(d, g.e[1]), D3(d, g.e[2]) };
    if (g.shape == SHAPE_PAWN)
    {
        float r = std::max(g.h[0], g.h[2]), rad = std::sqrt(q[0]*q[0] + q[2]*q[2]);
        if (std::fabs(q[1]) < g.h[1] && rad < r) return std::min(g.h[1]-std::fabs(q[1]), r-rad);
        return 0.0f;
    }
    float dm = 1e9f;
    for (int i = 0; i < 3; ++i) { if (std::fabs(q[i]) >= g.h[i]) return 0.0f; dm = std::min(dm, g.h[i]-std::fabs(q[i])); }
    return dm;
}
static float DepthInBox(const AABB& b, const glm::vec3& p)
{
    float dm = 1e9f;
    for (int i = 0; i < 3; ++i) { if (p[i] <= b.min[i] || p[i] >= b.max[i]) return 0.0f; dm = std::min(dm, std::min(p[i]-b.min[i], b.max[i]-p[i])); }
    return dm;
}
static void BodySamples(const Geom& g, std::vector<glm::vec3>& pts)
{
    pts.clear(); const int N = 13;
    if (g.shape == SHAPE_PAWN)
    {
        float r = std::max(g.h[0], g.h[2]);
        for (int j = 0; j <= N; ++j) for (int a = 0; a < 36; ++a)
        {
            float t = -g.h[1] + 2*g.h[1]*j/N, ang = 6.2831853f*a/36;
            pts.push_back(g.c + g.e[1]*t + g.e[0]*(r*std::cos(ang)) + g.e[2]*(r*std::sin(ang)));
            if (j == 0 || j == N) for (int s = 0; s < 4; ++s)
                pts.push_back(g.c + g.e[1]*t + g.e[0]*(r*s/4*std::cos(ang)) + g.e[2]*(r*s/4*std::sin(ang)));
        }
        return;
    }
    for (int f = 0; f < 3; ++f) for (int s = -1; s <= 1; s += 2)
    {
        int u = (f+1)%3, v = (f+2)%3;
        for (int i = 0; i <= N; ++i) for (int j = 0; j <= N; ++j)
            pts.push_back(g.c + g.e[f]*(s*g.h[f]) + g.e[u]*(-g.h[u] + 2*g.h[u]*i/N) + g.e[v]*(-g.h[v] + 2*g.h[v]*j/N));
    }
}
// Maior profundidade do corpo "idx" em qualquer caixa solida (menos ele).
static float Penetration(int idx, int* out_box_owner = 0)
{
    std::vector<AABB> boxes; std::vector<int> owner; BuildBoxes(boxes, owner);
    const Instance& in = g_Instances[idx];
    Geom g = MakeGeom(in);
    glm::vec3 a, b; GetMeshLocalBBox(in.mesh, &a, &b);
    AABB bb = AABBExpand(InstanceCollisionBox(in, idx, a, b), 0.01f);
    std::vector<glm::vec3> pts; BodySamples(g, pts);
    float worst = 0.0f;
    for (size_t k = 0; k < boxes.size(); ++k)
    {
        if (owner[k] == idx) continue;
        const AABB& B = boxes[k];
        if (!AABBOverlap(bb, B)) continue;
        float w = 0.0f;
        for (size_t i = 0; i < pts.size(); ++i) w = std::max(w, DepthInBox(B, pts[i]));
        // arestas da caixa recortadas a regiao do corpo
        for (int ax = 0; ax < 3; ++ax)
        {
            float lo = std::max(B.min[ax], bb.min[ax]), hi = std::min(B.max[ax], bb.max[ax]);
            if (hi <= lo) continue;
            int u = (ax+1)%3, v = (ax+2)%3;
            for (int cu = 0; cu < 2; ++cu) for (int cv = 0; cv < 2; ++cv)
            {
                glm::vec3 p; p[u] = cu ? B.max[u] : B.min[u]; p[v] = cv ? B.max[v] : B.min[v];
                int n = (int)((hi-lo)/0.002f) + 1;
                for (int s = 0; s <= n; ++s) { p[ax] = lo + (hi-lo)*s/n; w = std::max(w, DepthInBody(g, p)); }
            }
        }
        if (w > worst) { worst = w; if (out_box_owner) *out_box_owner = owner[k]; }
    }
    return worst;
}

// ---------------------------------------------------------------------------
static int FirstMesh(const char* n, int skip = 0)
{ for (size_t i = 0; i < g_Instances.size(); ++i) if (g_Instances[i].mesh == n && g_Instances[i].grabbable && skip-- == 0) return (int)i; return -1; }
static void Put(int i, glm::vec3 p, float s, float yaw = 0.0f)
{
    Instance& in = g_Instances[i]; in.pos = p; in.scale = glm::vec3(s); in.yaw = yaw; in.tilt = glm::mat4(1.0f);
    in.vel = glm::vec3(0.0f); in.body = BodyState();
}
static glm::vec3 Center(int i) { glm::vec3 a, b; GetMeshLocalBBox(g_Instances[i].mesh, &a, &b); return InstanceBodyCenter(g_Instances[i], a, b); }
static AABB Box(int i) { glm::vec3 a, b; GetMeshLocalBBox(g_Instances[i].mesh, &a, &b); return InstanceCollisionBox(i, a, b); }
static bool TiltIsCanonical(const Instance& in)
{
    for (int c = 0; c < 3; ++c) for (int r = 0; r < 3; ++r)
    { float v = std::fabs(in.tilt[c][r]); if (v > 1e-4f && std::fabs(v-1.0f) > 1e-4f) return false; }
    return true;
}
static const char* MotionName(int m)
{ static const char* n[] = { "REST", "FALL", "TIP", "SETTLE", "ROLL" }; return (m >= 0 && m <= 4) ? n[m] : "?"; }

struct Run
{
    float t_stop, max_pen, max_y_gain, release_angle, t_release;
    int   pen_owner; bool nan; std::string trace;
};
// Roda "seconds" de simulacao seguindo o objeto "idx". Registra as
// transicoes de estado, NaN, interpenetracao e se algum pegavel subiu.
static Run Simulate(int idx, float seconds, bool verbose = true)
{
    const float dt = PhysicsConfig::FIXED_STEP;
    Run R; R.t_stop = -1; R.max_pen = 0; R.max_y_gain = 0; R.pen_owner = -1; R.nan = false;
    R.release_angle = -1; R.t_release = -1;
    std::vector<float> y0; for (size_t i = 0; i < g_Instances.size(); ++i) y0.push_back(g_Instances[i].grabbable ? Center((int)i).y : 0.0f);
    int last = g_Instances[idx].body.motion; float last_angle = 0;
    char buf[160]; snprintf(buf, sizeof buf, "%s", MotionName(last)); R.trace = buf;
    glm::vec3 prev_pos = g_Instances[idx].pos; float still = 0;
    int nsteps = (int)(seconds / dt);
    for (int s = 0; s < nsteps; ++s)
    {
        Step(dt);
        float t = (s+1)*dt;
        const Instance& in = g_Instances[idx];
        if (in.body.motion == MOTION_TIP) last_angle = in.body.angle;
        if (in.body.motion != last)
        {
            if (last == MOTION_TIP && R.release_angle < 0) { R.release_angle = last_angle; R.t_release = t; }
            snprintf(buf, sizeof buf, " -> %s@%.3f", MotionName(in.body.motion), t); R.trace += buf; last = in.body.motion;
        }
        for (size_t i = 0; i < g_Instances.size(); ++i)
        {
            if (!g_Instances[i].grabbable) continue;
            if (!FiniteInst(g_Instances[i])) R.nan = true;
            R.max_y_gain = std::max(R.max_y_gain, Center((int)i).y - y0[i]);
        }
        int po = -1; float p = Penetration(idx, &po);
        if (p > R.max_pen) { R.max_pen = p; R.pen_owner = po; }
        float moved = std::fabs(in.pos.x-prev_pos.x) + std::fabs(in.pos.y-prev_pos.y) + std::fabs(in.pos.z-prev_pos.z);
        prev_pos = in.pos;
        if (in.body.motion == MOTION_REST && moved < 1e-7f) { still += dt; if (still >= 0.25f && R.t_stop < 0) R.t_stop = t - still + dt; }
        else { still = 0; R.t_stop = -1; }
    }
    if (verbose) printf("  estados: %s\n", R.trace.c_str());
    return R;
}

static void ResetScene() { g_Instances.clear(); BuildScene(g_Instances); }

int main()
{
    const char* env = getenv("TEST_USE_PHYSICS");
    g_use_physics = env && env[0] == '1';
    printf("driver: %s\n", g_use_physics ? "physics.cpp (UpdateFallingObjects)" : "driver proprio do teste (protocolo a4)");

    // =======================================================================
    // a) caso do usuario: dado 1.5 no botao, cubo magico 0.36 com o centro
    //    3,6 cm para fora da borda +X, apoiado em cima.
    // =======================================================================
    const float yaws[3] = { 0.0f, 0.40f, -0.75f };
    for (int vi = 0; vi < 4; ++vi)
    {
        ResetScene();
        int d0 = FirstMesh("the_die"), r = FirstMesh("the_rubik");
        Put(d0, glm::vec3(4.5f, 0.75f, 0.0f), 1.5f);
        float edge = 4.5f + 0.75f;
        float yaw = (vi < 3) ? yaws[vi] : 0.0f;
        float drop = (vi == 3) ? 0.3f : 0.0f;   // variante: solto 30 cm acima (como no t_base)
        Put(r, glm::vec3(edge + 0.036f, 1.5f + 0.18f, 0.0f), 0.36f, yaw);
        // com yaw a AABB cresce: apoia a base dela no topo do dado
        g_Instances[r].pos.y += 1.5f - Box(r).min.y + drop;
        float y_start = Center(r).y;
        printf("a%d) cubo 0.36 com centro 3.6 cm fora da borda +X do dado 1.5 (yaw %.2f%s)\n", vi, yaw, drop > 0 ? ", solto 0.3 m acima" : "");
        Run R = Simulate(r, 5.0f);
        const Instance& in = g_Instances[r];
        AABB b = Box(r);
        glm::vec3 c = Center(r);
        printf("  soltou da aresta apos %.1f graus (t=%.3f s); parou em %.3f s; pos final c=(%.3f, %.4f, %.3f) altura AABB %.4f\n",
               R.release_angle*57.2958f, R.t_release, R.t_stop, c.x, c.y, c.z, b.max.y - b.min.y);
        CHECK(R.release_angle > 0, "tombou (passou por TIP)");
        CHECK(R.release_angle > 0.80f && R.release_angle < 1.05f, "soltou pela separacao da face (45-60 graus): %.1f", R.release_angle*57.2958f);
        CHECK(std::fabs(g_Instances[d0].pos.x - 4.5f) < 1e-3f && std::fabs(g_Instances[d0].pos.z) < 1e-3f && std::fabs(g_Instances[d0].pos.y - 0.75f) < 1e-3f,
              "o dado de apoio nao se mexeu: (%.4f, %.4f, %.4f)", g_Instances[d0].pos.x, g_Instances[d0].pos.y, g_Instances[d0].pos.z);
        CHECK(c.x > edge + 0.18f && b.min.y < 1e-3f && b.min.y > -1e-3f, "caiu para +X e esta no chao (base y=%.4f, centro x=%.3f > %.3f)", b.min.y, c.x, edge + 0.18f);
        CHECK(std::fabs((b.max.y - b.min.y) - 0.36f) <= 1e-3f, "assentou numa face: altura AABB %.4f (0.36 +- 1 mm)", b.max.y - b.min.y);
        CHECK(TiltIsCanonical(in), "tilt com entradas 0/+-1");
        CHECK(in.body.motion == MOTION_REST && R.t_stop > 0, "terminou em REST (parou em %.3f s)", R.t_stop);
        CHECK(R.max_pen <= 3e-3f, "nao atravessou nada: penetracao maxima %.4f m (dona %d)", R.max_pen, R.pen_owner);
        CHECK(!R.nan, "sem NaN");
        CHECK(R.max_y_gain <= 1e-3f && c.y <= y_start + 1e-3f, "nenhum objeto subiu (max %.4f m)", R.max_y_gain);
        CHECK(b.max.x <= Room::HALF_X + 1e-3f, "nao entrou na parede +X (max.x %.4f)", b.max.x);
    }

    // =======================================================================
    // b) centro 3,6 cm para DENTRO: nao tomba.
    // =======================================================================
    {
        ResetScene();
        int d0 = FirstMesh("the_die"), r = FirstMesh("the_rubik");
        Put(d0, glm::vec3(4.5f, 0.75f, 0.0f), 1.5f);
        float edge = 4.5f + 0.75f;
        Put(r, glm::vec3(edge - 0.036f, 1.5f + 0.18f, 0.0f), 0.36f);
        printf("b) cubo com centro 3.6 cm para dentro da borda\n");
        Run R = Simulate(r, 2.0f);
        glm::vec3 c = Center(r);
        CHECK(R.release_angle < 0 && g_Instances[r].body.motion == MOTION_REST, "nao tombou (estado %s)", MotionName(g_Instances[r].body.motion));
        CHECK(std::fabs(c.y - 1.68f) < 1e-4f && std::fabs(c.x - (edge - 0.036f)) < 1e-4f, "ficou no lugar: c=(%.4f, %.4f)", c.x, c.y);
        CHECK(!R.nan && R.max_pen <= 1e-3f, "sem NaN, penetracao %.4f", R.max_pen);
    }

    // =======================================================================
    // c) cubo pendurado na borda com parede logo ao lado (dado 1 m encostado
    //    a 22 cm da parede -Z). Varia a folga entre o cubo e a parede.
    // =======================================================================
    {
        const float gaps[4] = { 0.0f, 0.01f, 0.03f, 0.10f };
        for (int gi = 0; gi < 4; ++gi)
        {
            ResetScene();
            int d0 = FirstMesh("the_die"), r = FirstMesh("the_rubik");
            float zc = -Room::HALF_Z + gaps[gi] + 0.18f;       // face -Z do cubo a "gap" da parede
            Put(d0, glm::vec3(5.0f, 0.5f, zc + 0.04f + 0.5f), 1.0f); // borda -Z do dado 4 cm para dentro do centro
            Put(r, glm::vec3(5.0f, 1.0f + 0.18f, zc), 0.36f);
            printf("c%d) cubo pendurado 4 cm na borda -Z do dado, folga ate a parede %.2f m\n", gi, gaps[gi]);
            Run R = Simulate(r, 3.0f);
            AABB b = Box(r);
            CHECK(b.min.z >= -Room::HALF_Z - 1e-3f, "nao atravessou a parede: z_min %.4f (parede -4.5)", b.min.z);
            CHECK(R.max_pen <= 3e-3f, "penetracao maxima %.4f (dona %d)", R.max_pen, R.pen_owner);
            CHECK(!R.nan && R.max_y_gain <= 1e-3f, "sem NaN, nada subiu (%.4f)", R.max_y_gain);
            printf("  final: estado %s c=(%.3f, %.3f, %.3f)\n", MotionName(g_Instances[r].body.motion), Center(r).x, Center(r).y, Center(r).z);
        }
        // c4) dado com vao de 0.5 m ate a parede: o cubo tomba para o vao,
        //     cai junto da parede e nao pode atravessa-la.
        ResetScene();
        int d0 = FirstMesh("the_die"), r = FirstMesh("the_rubik");
        Put(d0, glm::vec3(5.0f, 0.5f, -3.5f), 1.0f);           // z de -4.0 a -3.0
        Put(r, glm::vec3(5.0f, 1.0f + 0.18f, -4.0f - 0.05f), 0.36f);
        printf("c4) cubo tomba para um vao de 0.5 m entre o dado e a parede -Z\n");
        Run R = Simulate(r, 3.0f);
        AABB b = Box(r);
        CHECK(b.min.z >= -Room::HALF_Z - 1e-3f && R.max_pen <= 3e-3f, "nao atravessou: z_min %.4f, penetracao %.4f (dona %d)", b.min.z, R.max_pen, R.pen_owner);
        CHECK(!R.nan && R.max_y_gain <= 1e-3f && b.min.y < 1e-3f, "sem NaN, nada subiu, terminou no chao (base %.4f)", b.min.y);

        // c5) jogador logo ao lado: o cubo tomba, encosta nele e fica
        //     escorado; quando o jogador sai, termina de tombar e cai.
        ResetScene();
        d0 = FirstMesh("the_die"); r = FirstMesh("the_rubik");
        Put(d0, glm::vec3(0.0f, 0.5f, 0.0f), 1.0f);            // x de -0.5 a 0.5
        Put(r, glm::vec3(0.5f + 0.04f, 1.18f, 0.0f), 0.36f);   // face +X em x=0.72
        AABB saved = g_player;
        g_player.min = glm::vec3(0.78f, 0.0f, -0.2f); g_player.max = glm::vec3(1.18f, 1.8f, 0.2f);
        printf("c5) cubo tomba contra o jogador (6 cm de folga) e cai quando ele sai\n");
        Run R1 = Simulate(r, 1.0f);
        float lean = g_Instances[r].body.angle;
        AABB b1 = Box(r);
        CHECK(g_Instances[r].body.motion == MOTION_TIP && b1.max.x <= g_player.min.x + 1e-3f && R1.max_pen <= 3e-3f,
              "escorado no jogador: estado %s, angulo %.1f graus, max.x %.4f (jogador em %.2f)",
              MotionName(g_Instances[r].body.motion), lean*57.2958f, b1.max.x, g_player.min.x);
        g_player = saved;
        Run R2 = Simulate(r, 3.0f);
        AABB b2 = Box(r);
        CHECK(b2.min.y < 1e-3f && b2.min.y > -1e-3f && g_Instances[r].body.motion == MOTION_REST && TiltIsCanonical(g_Instances[r]),
              "jogador saiu: caiu e assentou no chao (base %.4f, estado %s)", b2.min.y, MotionName(g_Instances[r].body.motion));
        CHECK(!R1.nan && !R2.nan && R1.max_y_gain <= 1e-3f && R2.max_y_gain <= 1e-3f && R2.max_pen <= 3e-3f, "sem NaN, nada subiu, sem penetracao (%.4f)", R2.max_pen);
    }

    // =======================================================================
    // d) peao em pe na borda: tomba e termina deitado.
    // =======================================================================
    {
        const float over[2] = { 0.03f, 0.008f };
        for (int oi = 0; oi < 2; ++oi)
        {
            ResetScene();
            int d0 = FirstMesh("the_die"), pw = FirstMesh("the_pawn");
            Put(d0, glm::vec3(0.0f, 0.5f, 0.0f), 1.0f);       // x de -0.5 a 0.5
            Put(pw, glm::vec3(0.5f + over[oi], 1.0f + 0.28f, 0.0f), 0.56f);
            printf("d%d) peao 0.56 em pe com o centro %.1f cm fora da borda +X de um dado de 1 m\n", oi, over[oi]*100);
            float y_start = Center(pw).y;
            Run R = Simulate(pw, 4.0f);
            const Instance& in = g_Instances[pw];
            glm::vec3 u = InstanceLocalAxisWorld(in, 1);
            AABB b = Box(pw);
            printf("  soltou apos %.1f graus; eixo Y local em mundo (%.3f, %.3f, %.3f); base %.4f; c=(%.3f,%.3f,%.3f)\n",
                   R.release_angle*57.2958f, u.x, u.y, u.z, b.min.y, Center(pw).x, Center(pw).y, Center(pw).z);
            CHECK(R.release_angle > 0.80f && R.release_angle < 1.05f, "tombou e soltou pela separacao da face (%.1f graus)", R.release_angle*57.2958f);
            CHECK(std::fabs(u.y) < 1e-3f, "terminou deitado (|u.y| = %.5f)", std::fabs(u.y));
            CHECK(b.min.y > -1e-3f && b.min.y < 1e-3f, "apoiado no chao");
            CHECK(R.max_pen <= 3e-3f, "penetracao maxima %.4f (dona %d)", R.max_pen, R.pen_owner);
            CHECK(!R.nan && R.max_y_gain <= 1e-3f && Center(pw).y <= y_start, "sem NaN, nada subiu");
        }
    }

    // d2) o cenario b do t_m8 (a4): peao na borda +X do dado 1.5, com a
    //     parede +X a 0,75 m da borda. Cai rente ao dado e tem que deitar.
    {
        ResetScene();
        int d0 = FirstMesh("the_die"), pw = FirstMesh("the_pawn");
        Put(d0, glm::vec3(4.5f, 0.75f, 0.0f), 1.5f);
        Put(pw, glm::vec3(5.25f + 0.036f, 1.5f + 0.28f, 0.0f), 0.56f);
        printf("d2) peao 0.56 em pe 3.6 cm fora da borda +X do dado 1.5 (parede +X perto)\n");
        float y_start = Center(pw).y;
        Run R = Simulate(pw, 4.0f);
        glm::vec3 u = InstanceLocalAxisWorld(g_Instances[pw], 1);
        AABB b = Box(pw);
        printf("  soltou apos %.1f graus; u=(%.3f, %.3f, %.3f); base %.4f; c=(%.3f,%.3f,%.3f)\n",
               R.release_angle*57.2958f, u.x, u.y, u.z, b.min.y, Center(pw).x, Center(pw).y, Center(pw).z);
        CHECK(std::fabs(u.y) < 1e-3f && std::fabs(b.min.y) < 1e-3f, "terminou deitado no chao (|u.y| = %.5f)", std::fabs(u.y));
        CHECK(b.max.x <= Room::HALF_X + 1e-3f && R.max_pen <= 3e-3f, "nao entrou na parede (max.x %.4f), penetracao %.4f (dona %d)", b.max.x, R.max_pen, R.pen_owner);
        CHECK(!R.nan && R.max_y_gain <= 1e-3f && Center(pw).y <= y_start, "sem NaN, nada subiu");
    }

    // =======================================================================
    // e) varredura: varios objetos, escalas, overhangs e direcoes. Nenhum
    //    NaN, ninguem termina mais alto do que comecou, e os que eram caixa
    //    terminam numa face.
    // =======================================================================
    {
        printf("e) varredura (dado/cubo/tijolo/coelho, 4 direcoes, overhangs, escalas)\n");
        const char* meshes[4] = { "the_die", "the_rubik", "the_cube", "the_bunny" };
        int bad = 0, total = 0, tipped = 0; float worst_pen = 0; float worst_gain = 0; float worst_stop = 0;
        for (int mi = 0; mi < 4; ++mi)
        for (int dir = 0; dir < 4; ++dir)
        for (int oi = 0; oi < 3; ++oi)
        for (int si = 0; si < 2; ++si)
        {
            ResetScene();
            int d0 = FirstMesh("the_die", 0);
            int obj = (mi == 0) ? FirstMesh("the_die", 1) : FirstMesh(meshes[mi]);
            Put(d0, glm::vec3(0.0f, 0.4f, 0.0f), 0.8f);         // topo y=0.8, x,z de -0.4 a 0.4
            float s = (si == 0) ? 0.3f : 0.6f;
            if (mi == 3) s *= 0.5f;
            float ov = (oi == 0) ? 0.006f : (oi == 1 ? 0.04f : 0.12f) * s / 0.3f;
            glm::vec3 dirv[4] = { glm::vec3(1,0,0), glm::vec3(-1,0,0), glm::vec3(0,0,1), glm::vec3(0,0,-1) };
            glm::vec3 p = dirv[dir] * (0.4f + ov);
            Put(obj, glm::vec3(p.x, 2.0f, p.z), s, 0.1f*dir);
            g_Instances[obj].pos.y += 0.8f - Box(obj).min.y;
            float y_start = Center(obj).y;
            Run R = Simulate(obj, 4.0f, false);
            AABB b = Box(obj);
            bool ok = !R.nan && R.max_y_gain <= 1e-3f && Center(obj).y <= y_start + 1e-3f && R.max_pen <= 3e-3f
                      && TiltIsCanonical(g_Instances[obj]) && g_Instances[obj].body.motion == MOTION_REST;
            ++total; if (R.release_angle > 0) ++tipped;
            worst_pen = std::max(worst_pen, R.max_pen); worst_gain = std::max(worst_gain, R.max_y_gain);
            worst_stop = std::max(worst_stop, R.t_stop);
            if (!ok)
            {
                ++bad;
                printf("  FALHA %s s=%.2f dir=%d ov=%.3f: nan=%d ganho=%.4f pen=%.4f(dona %d) canon=%d estado=%s base=%.4f\n    %s\n",
                       meshes[mi], s, dir, ov, R.nan, R.max_y_gain, R.max_pen, R.pen_owner, TiltIsCanonical(g_Instances[obj]),
                       MotionName(g_Instances[obj].body.motion), b.min.y, R.trace.c_str());
            }
        }
        CHECK(bad == 0, "%d/%d casos ok (%d tombaram); pior penetracao %.4f m, maior subida %.4f m, maior tempo ate parar %.2f s",
              total - bad, total, tipped, worst_pen, worst_gain, worst_stop);
    }

    // e2) objeto grande quase equilibrado: o tombo livre mais lento. Tem que
    //     soltar pela separacao da face, nao pela seguranca MAX_TIP_TIME.
    //     Os outros pegaveis saem da cena para o cubo de 2 m cair livre.
    {
        ResetScene();
        int d0 = FirstMesh("the_die"), r = FirstMesh("the_rubik");
        for (size_t i = 0; i < g_Instances.size(); ++i)
            if (g_Instances[i].grabbable && (int)i != d0 && (int)i != r) { g_Instances[i].solid = false; g_Instances[i].grabbable = false; }
        Put(d0, glm::vec3(0.0f, 0.75f, 0.0f), 1.5f);
        Put(r, glm::vec3(0.0f, 1.5f + 1.0f, 0.75f + 0.006f), 2.0f);
        printf("e2) cubo de 2 m com o centro 6 mm fora da borda +Z de um dado de 1.5 m\n");
        float y_start = Center(r).y;
        Run R = Simulate(r, 6.0f);
        printf("  soltou apos %.1f graus em t=%.3f s (MAX_TIP_TIME %.1f); parou em %.2f s\n", R.release_angle*57.2958f, R.t_release, TipConfig::MAX_TIP_TIME, R.t_stop);
        CHECK(R.t_release > 0 && R.t_release < 0.6f*TipConfig::MAX_TIP_TIME && R.release_angle < TipConfig::RELEASE_ANGLE - 0.05f, "soltou pela separacao, longe da seguranca");
        CHECK(!R.nan && R.max_y_gain <= 1e-3f && Center(r).y <= y_start && R.max_pen <= 3e-3f && TiltIsCanonical(g_Instances[r]) && std::fabs(Box(r).min.y) < 1e-3f,
              "sem NaN, nada subiu, penetracao %.4f, face no chao (base %.4f)", R.max_pen, Box(r).min.y);
    }
    // e3) cubo de 2 m num dado de 0.8 m: a quina de fora chega no chao antes
    //     de a face largar a aresta. Fica escorado no chao e na aresta.
    {
        ResetScene();
        int d0 = FirstMesh("the_die"), r = FirstMesh("the_rubik");
        for (size_t i = 0; i < g_Instances.size(); ++i)
            if (g_Instances[i].grabbable && (int)i != d0 && (int)i != r) { g_Instances[i].solid = false; g_Instances[i].grabbable = false; }
        Put(d0, glm::vec3(0.0f, 0.4f, 0.0f), 0.8f);
        Put(r, glm::vec3(0.0f, 0.8f + 1.0f, 0.4f + 0.006f), 2.0f);
        printf("e3) cubo de 2 m num dado de 0.8 m: encosta a quina no chao\n");
        float y_start = Center(r).y;
        Run R = Simulate(r, 4.0f);
        printf("  estado final %s, angulo %.1f graus, base %.4f\n", MotionName(g_Instances[r].body.motion), g_Instances[r].body.angle*57.2958f, Box(r).min.y);
        CHECK(!R.nan && R.max_y_gain <= 1e-3f && Center(r).y <= y_start && R.max_pen <= 3e-3f && Box(r).min.y > -1e-3f,
              "sem NaN, nada subiu, nao entrou no chao (penetracao %.4f)", R.max_pen);
        CHECK(g_Instances[r].body.motion == MOTION_TIP && std::fabs(Box(r).min.y) < 2e-3f && std::fabs(g_Instances[d0].pos.z) < 1e-3f,
              "escorado no chao e na aresta (estado %s), dado parado (z %.4f)", MotionName(g_Instances[r].body.motion), g_Instances[d0].pos.z);
    }
    // e4) o apoio some no meio do tombo (jogador pegou o dado de baixo): cai.
    {
        ResetScene();
        int d0 = FirstMesh("the_die"), r = FirstMesh("the_rubik");
        Put(d0, glm::vec3(0.0f, 0.5f, 0.0f), 1.0f);
        Put(r, glm::vec3(0.5f + 0.02f, 1.18f, 0.0f), 0.36f);
        printf("e4) apoio retirado no meio do tombo\n");
        Simulate(r, 0.15f);
        float a = g_Instances[r].body.angle; int m0 = g_Instances[r].body.motion;
        g_Instances[d0].solid = false; g_Instances[d0].grabbable = false;   // "na mao": sai da lista de caixas
        float y_start = Center(r).y;
        Run R = Simulate(r, 2.0f);
        CHECK(m0 == MOTION_TIP && R.trace.find("FALL") != std::string::npos, "estava em TIP (%.1f graus) e passou a FALL", a*57.2958f);
        CHECK(std::fabs(Box(r).min.y) < 1e-3f && TiltIsCanonical(g_Instances[r]) && g_Instances[r].body.motion == MOTION_REST, "caiu e assentou no chao (base %.4f)", Box(r).min.y);
        CHECK(!R.nan && Center(r).y <= y_start && R.max_pen <= 3e-3f, "sem NaN, nao subiu, penetracao %.4f", R.max_pen);
    }

    printf("%s (%d falhas)\n", g_fail ? "FALHOU" : "TUDO OK", g_fail);
    return g_fail ? 1 : 0;
}
