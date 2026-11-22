// Testes do a3-roll (roll.cpp: bola e peao deitado rolando).
// Rodar: tools/check.sh test/t_a3.cpp
//
// Cada cenario roda com dois "passos" de fisica:
//   - OwnStep: laco minimo deste teste (queda vertical, apoio/equilibrio,
//     RollOnLand/RollStep, ResolveAABBMovement, RollAfterMove, giro no ar),
//     seguindo o protocolo combinado com a4-integ;
//   - UpdateFallingObjects (physics.cpp), quando ele ja chama roll.cpp.
//     Desligue com A3_OWN_ONLY=1 no ambiente.
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include "scene.h"
#include "rigid.h"
#include "roll.h"
#include "tip.h"
#include "physics.h"
#include "grab.h"
#include "matrices.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { if (cond) ++g_pass; else { ++g_fail; printf("  FALHOU %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const float DT = PhysicsConfig::FIXED_STEP;
static const float PI = 3.14159265358979f;
static bool g_UsePhysics = false;   // false: OwnStep; true: UpdateFallingObjects

// ---------------------------------------------------------------------------
// utilitarios
// ---------------------------------------------------------------------------
static float OrthoErr(const glm::mat4& R)
{
    float e = 0;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
    {
        float d = R[i][0]*R[j][0] + R[i][1]*R[j][1] + R[i][2]*R[j][2];
        e = std::max(e, std::fabs(d - (i == j ? 1.0f : 0.0f)));
    }
    return e;
}
static float Det3(const glm::mat4& R)
{
    return R[0][0]*(R[1][1]*R[2][2]-R[2][1]*R[1][2]) - R[1][0]*(R[0][1]*R[2][2]-R[2][1]*R[0][2]) + R[2][0]*(R[0][1]*R[1][2]-R[1][1]*R[0][2]);
}
static bool Finite(float x) { return x == x && std::fabs(x) < 1e30f; }
static bool InstFinite(const Instance& in)
{
    for (int i = 0; i < 3; ++i)
        if (!Finite(in.pos[i]) || !Finite(in.vel[i]) || !Finite(in.body.ang_vel[i])) return false;
    for (int c = 0; c < 4; ++c) for (int r = 0; r < 4; ++r) if (!Finite(in.tilt[c][r])) return false;
    return Finite(in.yaw);
}
static float HLen(const glm::vec3& v) { return std::sqrt(v.x*v.x + v.z*v.z); }

// Angulo e eixo da rotacao relativa B * A^T (de A para B), pela parte
// antissimetrica (bom para angulos pequenos) e pelo traco.
static float RelAngle(const glm::mat4& A, const glm::mat4& B, glm::vec3* axis)
{
    // M = B * A^T  (3x3).  M[c][r] = sum_k B[k][r] * A[k][c]
    float M[3][3];
    for (int c = 0; c < 3; ++c) for (int r = 0; r < 3; ++r)
    {
        float s = 0; for (int k = 0; k < 3; ++k) s += B[k][r] * A[k][c];
        M[c][r] = s;
    }
    // vee = (M_zy - M_yz, M_xz - M_zx, M_yx - M_xy)/2, com M_rc = M[c][r]
    glm::vec3 v(0.5f*(M[1][2] - M[2][1]), 0.5f*(M[2][0] - M[0][2]), 0.5f*(M[0][1] - M[1][0]));
    float s = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    float c = 0.5f*(M[0][0] + M[1][1] + M[2][2] - 1.0f);
    if (axis) *axis = (s > 1e-12f) ? v * (1.0f/s) : glm::vec3(0);
    return std::atan2(s, c);
}

static glm::vec3 LB(const Instance& in, glm::vec3* hi)
{
    glm::vec3 a, b; GetMeshLocalBBox(in.mesh, &a, &b); if (hi) *hi = b; return a;
}
static AABB BoxOf(int i)
{
    glm::vec3 b; glm::vec3 a = LB(g_Instances[i], &b);
    return InstanceCollisionBox(i, a, b);
}
static glm::vec3 CenterOf(int i)
{
    glm::vec3 b; glm::vec3 a = LB(g_Instances[i], &b);
    return InstanceBodyCenter(g_Instances[i], a, b);
}

// ---------------------------------------------------------------------------
// cena de teste
// ---------------------------------------------------------------------------
static Instance SBox(glm::vec3 c, glm::vec3 size)
{
    Instance i;
    i.mesh = "the_cube"; i.pos = c; i.yaw = 0; i.scale = size;
    i.mat = MakeMaterial(0, UV_OBJ, 1.0f); i.solid = true; i.grabbable = false; i.casts_shadow = false;
    return i;
}
static Instance Obj(const char* mesh, int shape, float s, glm::vec3 p)
{
    Instance i;
    i.mesh = mesh; i.pos = p; i.yaw = 0; i.scale = glm::vec3(s);
    i.mat = MakeMaterial(0, UV_OBJ, 1.0f); i.solid = true; i.grabbable = true; i.casts_shadow = true;
    i.shape = shape;
    return i;
}
// piso grande (topo em y = 0) e, opcional, paredes formando um cercado
static void NewScene(bool walls, float hx = 3.0f, float hz = 3.0f)
{
    g_Instances.clear();
    g_DoorInstance = -1; g_ButtonInstance = -1; g_BezierInstance = -1;
    g_Instances.push_back(SBox(glm::vec3(0, -0.1f, 0), glm::vec3(40, 0.2f, 40)));
    if (walls)
    {
        g_Instances.push_back(SBox(glm::vec3( hx + 0.1f, 1, 0), glm::vec3(0.2f, 2, 2*hz + 0.4f)));
        g_Instances.push_back(SBox(glm::vec3(-hx - 0.1f, 1, 0), glm::vec3(0.2f, 2, 2*hz + 0.4f)));
        g_Instances.push_back(SBox(glm::vec3(0, 1,  hz + 0.1f), glm::vec3(2*hx + 0.4f, 2, 0.2f)));
        g_Instances.push_back(SBox(glm::vec3(0, 1, -hz - 0.1f), glm::vec3(2*hx + 0.4f, 2, 0.2f)));
    }
}
static int Add(const Instance& in) { g_Instances.push_back(in); return (int)g_Instances.size() - 1; }

// peao deitado com o eixo (base -> cabeca) na direcao +X: Rz(-90) leva Y em X
static void LayPawnAlongX(Instance& p)
{
    p.tilt = Matrix_Rotate_Z(-0.5f * PI);
}

// ---------------------------------------------------------------------------
// Laco de um passo do proprio teste (protocolo combinado com a4-integ).
// ---------------------------------------------------------------------------
static void OwnStep(float dt)
{
    std::vector<AABB> boxes; std::vector<int> owner;
    for (size_t i = 0; i < g_Instances.size(); ++i)
    {
        if (!g_Instances[i].solid) continue;
        boxes.push_back(BoxOf((int)i)); owner.push_back((int)i);
    }
    AABB player; player.min = glm::vec3(100); player.max = glm::vec3(101);
    boxes.push_back(player); owner.push_back(-1);

    std::vector<size_t> order;
    for (size_t k = 0; k < boxes.size(); ++k)
        if (owner[k] >= 0 && g_Instances[owner[k]].grabbable) order.push_back(k);
    std::sort(order.begin(), order.end(), [&boxes](size_t a, size_t b) { return boxes[a].min.y < boxes[b].min.y; });

    for (size_t m = 0; m < order.size(); ++m)
    {
        size_t k = order[m]; int idx = owner[k];
        Instance& inst = g_Instances[idx];
        glm::vec3 lmin, lmax; GetMeshLocalBBox(inst.mesh, &lmin, &lmax);
        PhysicsContext ctx; ctx.boxes = &boxes; ctx.owner = &owner; ctx.self = (int)k;
        ctx.instance = idx; ctx.lmin = lmin; ctx.lmax = lmax;
        AABB& box = boxes[k];

        bool was_air = (inst.body.motion == MOTION_FALL);

        // vertical (M7)
        float v = inst.vel.y - PhysicsConfig::GRAVITY * dt;
        if (v < -PhysicsConfig::MAX_FALL_SPEED) v = -PhysicsConfig::MAX_FALL_SPEED;
        float dy = v * dt, support = 0; int support_k = -1; bool supported = false;
        if (FindSupportHeight(box, boxes, (int)k, PhysicsConfig::SUPPORT_TOL, &support, &support_k)
            && box.min.y + dy <= support)
        {
            supported = true; dy = support - box.min.y; v = 0;
        }
        inst.pos.y += dy; box.min.y += dy; box.max.y += dy; inst.vel.y = v;

        if (!supported)
        {
            inst.body.motion = MOTION_FALL;
            // giro no ar com ang_vel (o que AirSpinStep faz)
            glm::vec3 w = inst.body.ang_vel;
            float wl = std::sqrt(w.x*w.x + w.y*w.y + w.z*w.z);
            if (wl > 1e-6f)
            {
                RotateAboutCenter(inst, lmin, lmax, w * (1.0f/wl), wl * dt);
                box = InstanceCollisionBox(inst, idx, lmin, lmax);
            }
        }
        else
        {
            if (was_air)
            {
                if (RollsNow(inst)) RollOnLand(inst, ctx);
                else { inst.vel.x = inst.vel.z = 0; inst.body.motion = MOTION_REST; inst.body.ang_vel = glm::vec3(0); }
            }

            SupportInfo sup; sup.supported = true; sup.height = support; sup.box_index = support_k;
            sup.unbalanced = false; sup.overhang = glm::vec3(0);
            sup.fx0 = sup.fx1 = sup.fz0 = sup.fz1 = 0;
            if (SupportFootprint(box, boxes, (int)k, support, PhysicsConfig::SUPPORT_TOL,
                                 &sup.fx0, &sup.fx1, &sup.fz0, &sup.fz1))
            {
                glm::vec3 c = InstanceBodyCenter(inst, lmin, lmax);
                const float mg = PhysicsConfig::BALANCE_MARGIN;
                float ox = 0, oz = 0;
                if      (c.x < sup.fx0 - mg) ox = c.x - sup.fx0;
                else if (c.x > sup.fx1 + mg) ox = c.x - sup.fx1;
                if      (c.z < sup.fz0 - mg) oz = c.z - sup.fz0;
                else if (c.z > sup.fz1 + mg) oz = c.z - sup.fz1;
                if (ox != 0 || oz != 0) { sup.unbalanced = true; sup.overhang = glm::vec3(ox, 0, oz); }
            }

            if (sup.unbalanced)
            {
                if (RollsNow(inst) && RollHandlesOverhang(inst, sup.overhang))
                { inst.body.motion = MOTION_ROLL; RollStep(inst, ctx, sup, dt); }
                else { inst.vel.x = inst.vel.z = 0; }
            }
            else if (inst.body.motion == MOTION_ROLL) RollStep(inst, ctx, sup, dt);
            else { inst.vel.x = inst.vel.z = 0; }
        }

        if (inst.vel.x != 0 || inst.vel.z != 0)
        {
            glm::vec3 delta(inst.vel.x * dt, 0, inst.vel.z * dt);
            bool hit[3] = { false, false, false };
            glm::vec3 applied = ResolveAABBMovement(box, delta, boxes, (int)k, hit);
            inst.pos.x += applied.x; inst.pos.z += applied.z;
            box.min.x += applied.x; box.max.x += applied.x;
            box.min.z += applied.z; box.max.z += applied.z;
            if (RollsNow(inst) && (inst.body.motion == MOTION_ROLL || inst.body.motion == MOTION_FALL))
                RollAfterMove(inst, ctx, applied, hit, dt);
            else { if (hit[0]) inst.vel.x = 0; if (hit[2]) inst.vel.z = 0; }
        }
    }
}

static void Step(float dt)
{
    if (g_UsePhysics)
    {
        AABB player; player.min = glm::vec3(100); player.max = glm::vec3(101);
        UpdateFallingObjects(-1, player, dt);
    }
    else
        OwnStep(dt);
}

// ---------------------------------------------------------------------------
// 0) funcoes isoladas
// ---------------------------------------------------------------------------
static void TestUnits()
{
    printf("-- 0) funcoes isoladas\n");
    NewScene(false);
    int b  = Add(Obj("the_sphere", SHAPE_SPHERE, 0.18f, glm::vec3(0, 0.18f, 0)));
    int pw = Add(Obj("the_pawn",   SHAPE_PAWN,   0.56f, glm::vec3(1, 0.28f, 0)));
    int cb = Add(Obj("the_cube",   SHAPE_BOX,    0.36f, glm::vec3(-1, 0.18f, 0)));
    CHECK(RollsNow(g_Instances[b]),   "bola deveria rolar");
    CHECK(!RollsNow(g_Instances[pw]), "peao em pe nao rola");
    CHECK(!RollsNow(g_Instances[cb]), "cubo nao rola");
    LayPawnAlongX(g_Instances[pw]);
    CHECK(RollsNow(g_Instances[pw]),  "peao deitado rola");
    CHECK(RollHandlesOverhang(g_Instances[b], glm::vec3(0.05f, 0, 0.02f)), "bola rola para qualquer lado");
    CHECK(RollHandlesOverhang(g_Instances[pw], glm::vec3(0, 0, 0.05f)),   "peao (eixo X) pendurado em Z rola");
    CHECK(!RollHandlesOverhang(g_Instances[pw], glm::vec3(0.05f, 0, 0)),  "peao (eixo X) pendurado em X tomba");
    CHECK(!RollHandlesOverhang(g_Instances[pw], glm::vec3(-0.05f, 0, 0.02f)), "peao pendurado mais pela ponta tomba");

    // sinal do giro: bola indo para +X gira em torno de -Z (o topo vai para +X)
    {
        glm::vec3 lmin, lmax; GetMeshLocalBBox("the_sphere", &lmin, &lmax);
        std::vector<AABB> boxes; std::vector<int> owner;
        boxes.push_back(BoxOf(0)); owner.push_back(0);
        boxes.push_back(BoxOf(b)); owner.push_back(b);
        PhysicsContext ctx; ctx.boxes = &boxes; ctx.owner = &owner; ctx.self = 1; ctx.instance = b;
        ctx.lmin = lmin; ctx.lmax = lmax;
        Instance& in = g_Instances[b];
        in.body.motion = MOTION_ROLL; in.vel = glm::vec3(1, 0, 0);
        glm::mat4 O0 = InstanceOrientation(in);
        glm::vec3 d(0.01f, 0, 0); bool hit[3] = { false, false, false };
        in.pos += d;
        RollAfterMove(in, ctx, d, hit, DT);
        glm::vec3 ax; float ang = RelAngle(O0, InstanceOrientation(in), &ax);
        CHECK(std::fabs(ang - 0.01f/0.18f) < 1e-4f, "angulo %.5f esperado %.5f", ang, 0.01f/0.18f);
        CHECK(ax.z < -0.999f, "eixo de giro (%.3f %.3f %.3f) deveria ser -Z", ax.x, ax.y, ax.z);
        glm::vec3 top = InstanceLocalAxisWorld(in, 1);
        CHECK(top.x > 0, "topo da bola deveria ir para +X (x=%.4f)", top.x);
        CHECK(std::fabs(in.body.ang_vel.z + 1.0f/0.18f) < 1e-3f && std::fabs(in.body.ang_vel.x) < 1e-6f,
              "ang_vel (%.3f %.3f %.3f) deveria ser (0,0,-v/r)", in.body.ang_vel.x, in.body.ang_vel.y, in.body.ang_vel.z);
        glm::vec3 c = CenterOf(b);
        CHECK(std::fabs(c.x - 0.01f) < 1e-5f && std::fabs(c.y - 0.18f) < 1e-5f, "centro nao deveria mudar com o giro");

        // pouso de bola que saiu rolando da borda: mantem LAND_KEEP de v
        in.vel = glm::vec3(1.2f, -2.0f, 0); in.body.ang_vel = glm::vec3(0, 0, -1.2f/0.18f);
        in.body.motion = MOTION_FALL;
        RollOnLand(in, ctx);
        CHECK(in.body.motion == MOTION_ROLL, "pouso rolando deveria ser ROLL");
        CHECK(std::fabs(in.vel.x - RollConfig::LAND_KEEP*1.2f) < 1e-4f && std::fabs(in.vel.z) < 1e-6f,
              "pouso com giro: vx=%.4f esperado %.4f", in.vel.x, RollConfig::LAND_KEEP*1.2f);
        // sem giro (escorregando): o atrito tira parte da velocidade (5/7)
        in.vel = glm::vec3(1.2f, -2.0f, 0); in.body.ang_vel = glm::vec3(0); in.body.motion = MOTION_FALL;
        RollOnLand(in, ctx);
        float exp = RollConfig::LAND_KEEP * 1.2f / (1.0f + RollConfig::SPHERE_INERTIA);
        CHECK(std::fabs(in.vel.x - exp) < 1e-4f, "pouso sem giro: vx=%.4f esperado %.4f", in.vel.x, exp);
        // solta parada: REST
        in.vel = glm::vec3(0, -3, 0); in.body.ang_vel = glm::vec3(0); in.body.motion = MOTION_FALL;
        RollOnLand(in, ctx);
        CHECK(in.body.motion == MOTION_REST && in.vel.x == 0 && in.vel.z == 0, "bola solta parada deveria ficar REST");
    }

    // pouso do peao deitado (eixo X)
    {
        glm::vec3 lmin, lmax; GetMeshLocalBBox("the_pawn", &lmin, &lmax);
        Instance& p = g_Instances[pw];
        p.pos = glm::vec3(1, 0.335f*0.56f, 0);
        std::vector<AABB> boxes; std::vector<int> owner;
        boxes.push_back(BoxOf(0)); owner.push_back(0);
        boxes.push_back(BoxOf(pw)); owner.push_back(pw);
        PhysicsContext ctx; ctx.boxes = &boxes; ctx.owner = &owner; ctx.self = 1; ctx.instance = pw;
        ctx.lmin = lmin; ctx.lmax = lmax;
        // velocidade so ao longo do eixo (caiu de ponta): nao rola
        p.vel = glm::vec3(1.0f, -2, 0); p.body.ang_vel = glm::vec3(0, 0, 3.0f); p.body.motion = MOTION_FALL;
        RollOnLand(p, ctx);
        CHECK(p.body.motion == MOTION_REST && p.vel.x == 0 && p.vel.z == 0, "peao caindo ao longo do eixo nao rola");
        // velocidade perpendicular com giro coerente em torno do eixo
        float r = 0.335f*0.56f;
        // rolando para +Z: omega = (up x v)/r = (+X)/r
        p.vel = glm::vec3(0.3f, -2, 1.0f); p.body.ang_vel = glm::vec3(1.0f/r, 0, 0); p.body.motion = MOTION_FALL;
        RollOnLand(p, ctx);
        CHECK(p.body.motion == MOTION_ROLL, "peao rolando ao pousar: ROLL");
        CHECK(std::fabs(p.vel.x) < 1e-5f && std::fabs(p.vel.z - RollConfig::LAND_KEEP) < 1e-3f,
              "peao pouso: vel (%.4f, %.4f) esperado (0, %.3f)", p.vel.x, p.vel.z, RollConfig::LAND_KEEP);
        // giro sozinho (sem velocidade) vira velocidade de rolamento: giro em
        // torno de +X com up x v / r = w  =>  v = r (w x up) em +Z
        p.vel = glm::vec3(0, -2, 0); p.body.ang_vel = glm::vec3(2.0f, 0, 0); p.body.motion = MOTION_FALL;
        RollOnLand(p, ctx);
        float vexp = RollConfig::LAND_KEEP * RollConfig::PAWN_INERTIA * 2.0f * r / (1 + RollConfig::PAWN_INERTIA);
        CHECK(std::fabs(p.vel.z - vexp) < 1e-4f && std::fabs(p.vel.x) < 1e-6f,
              "giro vira rolamento: vz=%.4f esperado %.4f", p.vel.z, vexp);

        // no ar (FALL) RollAfterMove so faz o quique: o peao que tombou de
        // uma caixa voa com a velocidade do tombo ao longo do eixo e nao
        // pode perde-la (achado do b1-step na integracao com tip.cpp)
        p.vel = glm::vec3(0.75f, -1.27f, 0.2f); p.body.motion = MOTION_FALL;
        glm::vec3 d(0.75f*DT, 0, 0.2f*DT); bool nohit[3] = { false, false, false };
        RollAfterMove(p, ctx, d, nohit, DT);
        CHECK(std::fabs(p.vel.x - 0.75f) < 1e-6f && std::fabs(p.vel.z - 0.2f) < 1e-6f,
              "peao no ar perdeu velocidade: (%.4f, %.4f)", p.vel.x, p.vel.z);
        bool hitx[3] = { true, false, false };
        RollAfterMove(p, ctx, d, hitx, DT);
        CHECK(std::fabs(p.vel.x + RollConfig::WALL_RESTITUTION*0.75f) < 1e-6f && std::fabs(p.vel.z - 0.2f) < 1e-6f,
              "peao no ar batendo em X: (%.4f, %.4f)", p.vel.x, p.vel.z);
    }
}

// ---------------------------------------------------------------------------
// a) bola parada no piso fica parada (10 s)
// ---------------------------------------------------------------------------
static void TestRest()
{
    printf("-- a) bola parada no piso, 10 s\n");
    NewScene(false);
    int b = Add(Obj("the_sphere", SHAPE_SPHERE, 0.18f, glm::vec3(0.3f, 0.18f, -0.4f)));
    int p = Add(Obj("the_pawn",   SHAPE_PAWN,   0.56f, glm::vec3(1.5f, 0.335f*0.56f, 0.5f)));
    LayPawnAlongX(g_Instances[p]);
    Instance b0 = g_Instances[b], p0 = g_Instances[p];
    int n = (int)(10.0f / DT); bool moved = false;
    for (int i = 0; i < n; ++i) { Step(DT); if (g_Instances[b].body.motion != MOTION_REST) moved = true; }
    const Instance& B = g_Instances[b]; const Instance& P = g_Instances[p];
    float db = HLen(B.pos - b0.pos) + std::fabs(B.pos.y - b0.pos.y);
    float dp = HLen(P.pos - p0.pos) + std::fabs(P.pos.y - p0.pos.y);
    float rot = RelAngle(InstanceOrientation(b0), InstanceOrientation(B), 0);
    printf("   deriva bola %.2e m, giro %.2e rad; peao deitado %.2e m\n", db, rot, dp);
    CHECK(db < 1e-5f && rot < 1e-5f && !moved, "bola derivou (%.2e m, %.2e rad)", db, rot);
    CHECK(dp < 1e-5f, "peao deitado derivou %.2e", dp);
}

// AirSpinStep real (tip.cpp) ou o fraco de test/weak_m8.cpp? No laco
// proprio o giro no ar e feito pelo teste, entao sempre "funciona".
static bool AirSpinWorks()
{
    if (!g_UsePhysics) return true;
    std::vector<Instance> saved = g_Instances;
    NewScene(false);
    int b = Add(Obj("the_sphere", SHAPE_SPHERE, 0.18f, glm::vec3(0, 2.0f, 0)));
    glm::vec3 lmin, lmax; GetMeshLocalBBox("the_sphere", &lmin, &lmax);
    std::vector<AABB> boxes; std::vector<int> owner;
    boxes.push_back(BoxOf(0)); owner.push_back(0);
    boxes.push_back(BoxOf(b)); owner.push_back(b);
    PhysicsContext ctx; ctx.boxes = &boxes; ctx.owner = &owner; ctx.self = 1; ctx.instance = b;
    ctx.lmin = lmin; ctx.lmax = lmax;
    g_Instances[b].body.motion = MOTION_FALL;
    g_Instances[b].body.ang_vel = glm::vec3(0, 0, -5.0f);
    glm::mat4 O0 = InstanceOrientation(g_Instances[b]);
    AirSpinStep(g_Instances[b], ctx, DT);
    bool works = RelAngle(O0, InstanceOrientation(g_Instances[b]), 0) > 1e-4f;
    g_Instances = saved;
    return works;
}

// ---------------------------------------------------------------------------
// b) bola com o centro alem da borda de uma caixa
// ---------------------------------------------------------------------------
static void TestEdge()
{
    printf("-- b) bola com o centro alem da borda de uma caixa\n");
    NewScene(false);
    Add(SBox(glm::vec3(0, 0.5f, 0), glm::vec3(1.0f)));          // caixa: x em [-0.5, 0.5], topo y = 1
    const float r = 0.18f;
    int b = Add(Obj("the_sphere", SHAPE_SPHERE, r, glm::vec3(0.52f, 1.0f + r, 0.1f)));

    float t = 0, dist = 0, ang = 0, worst_axis = 1, max_x = 0, dist_roll = 0, ang_roll = 0;
    bool fell = false, rolled_floor = false, stopped = false, leftbox_rolling = false;
    float t_stop = 0;
    glm::vec3 c_prev = CenterOf(b);
    glm::mat4 O_prev = InstanceOrientation(g_Instances[b]);
    for (int i = 0; i < (int)(20.0f / DT); ++i)
    {
        Step(DT); t += DT;
        const Instance& B = g_Instances[b];
        glm::vec3 c = CenterOf(b);
        glm::mat4 O = InstanceOrientation(B);
        glm::vec3 d = c - c_prev; d.y = 0;
        glm::vec3 ax; float a = RelAngle(O_prev, O, &ax);
        float dl = HLen(d);
        dist += dl; ang += a;
        if (B.body.motion == MOTION_ROLL) { dist_roll += dl; ang_roll += a; }
        if (a > 1e-6f && dl > 1e-6f)
        {
            // eixo esperado: up x d / |d| = (dz, 0, -dx)/|d|
            glm::vec3 e(d.z/dl, 0, -d.x/dl);
            worst_axis = std::min(worst_axis, ax.x*e.x + ax.y*e.y + ax.z*e.z);
        }
        if (B.body.motion == MOTION_ROLL && c.x > 0.5f && c.y > 1.0f) leftbox_rolling = true;
        if (B.body.motion == MOTION_FALL) fell = true;
        if (fell && B.body.motion == MOTION_ROLL && c.y < r + 1e-3f) rolled_floor = true;
        if (rolled_floor && B.body.motion == MOTION_REST && !stopped) { stopped = true; t_stop = t; }
        max_x = std::max(max_x, c.x);
        c_prev = c; O_prev = O;
        if (!InstFinite(B)) { CHECK(false, "NaN na bola"); break; }
    }
    const Instance& B = g_Instances[b];
    glm::vec3 c = CenterOf(b);
    printf("   parou em %.2f s em x=%.3f y=%.3f z=%.3f; distancia %.3f m, giro %.3f rad, dist/r %.3f\n",
           t_stop, c.x, c.y, c.z, dist, ang, dist / r);
    CHECK(leftbox_rolling, "deveria rolar para fora da borda");
    CHECK(fell, "deveria cair");
    CHECK(rolled_floor, "deveria seguir rolando no piso");
    CHECK(stopped && B.body.motion == MOTION_REST && B.vel.x == 0 && B.vel.z == 0, "deveria parar (REST)");
    CHECK(std::fabs(c.y - r) < 1e-4f, "deveria estar apoiada no piso, y=%.4f", c.y);
    CHECK(c.x > 0.5f + r + 0.3f, "deveria ter ido para +X, x=%.3f", c.x);
    CHECK(std::fabs(c.z - 0.1f) < 1e-3f, "nao deveria desviar em z (%.4f)", c.z);
    printf("   so rolando (ROLL): distancia %.3f m, giro %.3f rad, dist/r %.3f\n", dist_roll, ang_roll, dist_roll / r);
    CHECK(std::fabs(ang_roll - dist_roll / r) < 0.01f * (dist_roll / r), "giro rolando %.3f deveria ser distancia/raio %.3f", ang_roll, dist_roll / r);
    // a conta total inclui o voo, em que quem gira e AirSpinStep (tip.cpp)
    if (AirSpinWorks())
        CHECK(std::fabs(ang - dist / r) < 0.02f * (dist / r), "giro total %.3f deveria ser ~ distancia/raio %.3f", ang, dist / r);
    else
        printf("   (AirSpinStep ainda e o fraco: giro no ar nao conferido)\n");
    CHECK(worst_axis > 0.999f, "eixo de giro incoerente com o movimento (cos %.4f)", worst_axis);
    CHECK(OrthoErr(InstanceOrientation(B)) < 1e-4f, "orientacao nao ortonormal");
    // depois de parar, fica parada
    glm::vec3 cs = c;
    for (int i = 0; i < 480; ++i) Step(DT);
    CHECK(HLen(CenterOf(b) - cs) < 1e-6f, "andou depois de parar");
}

// ---------------------------------------------------------------------------
// c) bola rolando contra a parede
// ---------------------------------------------------------------------------
static void TestWall()
{
    printf("-- c) bola rolando contra a parede\n");
    NewScene(true, 3.0f, 3.0f);                     // parede +X com face em x = 3
    const float r = 0.18f;
    int b = Add(Obj("the_sphere", SHAPE_SPHERE, r, glm::vec3(1.5f, r, 0.4f)));
    g_Instances[b].vel = glm::vec3(2.5f, 0, 0.3f);
    g_Instances[b].body.motion = MOTION_ROLL;

    float vx_hit = 0, vx_after = 0, max_x = -10; bool bounced = false; float prev_vx = 2.5f;
    for (int i = 0; i < (int)(15.0f / DT); ++i)
    {
        Step(DT);
        const Instance& B = g_Instances[b];
        AABB bb = BoxOf(b);
        max_x = std::max(max_x, bb.max.x);
        if (!bounced && B.vel.x < 0) { bounced = true; vx_hit = prev_vx; vx_after = B.vel.x; }
        prev_vx = B.vel.x;
        if (!InstFinite(B)) { CHECK(false, "NaN"); break; }
    }
    const Instance& B = g_Instances[b];
    printf("   bateu com vx=%.3f, voltou com vx=%.3f (razao %.3f); x max da caixa %.5f (parede 3.0); final x=%.3f, estado %d\n",
           vx_hit, vx_after, vx_hit != 0 ? -vx_after / vx_hit : 0.0f, max_x, CenterOf(b).x, B.body.motion);
    CHECK(bounced, "deveria voltar");
    CHECK(std::fabs(-vx_after / vx_hit - RollConfig::WALL_RESTITUTION) < 0.05f, "razao do quique errada");
    CHECK(max_x <= 3.0f + 1e-4f, "atravessou a parede (%.5f)", max_x);
    CHECK(CenterOf(b).x < 3.0f - r - 0.05f, "deveria ter se afastado da parede");
    CHECK(B.body.motion == MOTION_REST, "deveria parar no fim");
    CHECK(std::fabs(CenterOf(b).y - r) < 1e-4f, "altura errada");
}

// ---------------------------------------------------------------------------
// d) peao deitado com velocidade inicial
// ---------------------------------------------------------------------------
static void TestPawn()
{
    printf("-- d) peao deitado rolando\n");
    NewScene(false);
    const float s = 0.56f, r = 0.335f * s;
    int p = Add(Obj("the_pawn", SHAPE_PAWN, s, glm::vec3(0, r, 0)));
    LayPawnAlongX(g_Instances[p]);                  // eixo (base -> cabeca) em +X
    g_Instances[p].vel = glm::vec3(0, 0, 1.5f);      // perpendicular ao eixo
    g_Instances[p].body.motion = MOTION_ROLL;

    AABB b0 = BoxOf(p);
    float h0 = b0.max.y - b0.min.y, worst_h = 0, worst_par = 0, dist = 0, spin = 0, worst_axis_y = 0;
    float yaw0 = g_Instances[p].yaw; float t = 0; bool stopped = false;
    glm::vec3 c_prev = CenterOf(p), c0 = c_prev;
    glm::mat4 O_prev = InstanceOrientation(g_Instances[p]);
    glm::vec3 a_prev = InstanceLocalAxisWorld(g_Instances[p], 1);
    float yaw_total = 0;
    for (int i = 0; i < (int)(10.0f / DT); ++i)
    {
        Step(DT); t += DT;
        Instance& P = g_Instances[p];
        AABB bb = BoxOf(p);
        worst_h = std::max(worst_h, std::fabs((bb.max.y - bb.min.y) - h0));
        worst_h = std::max(worst_h, std::fabs(bb.min.y));
        glm::vec3 a = InstanceLocalAxisWorld(P, 1);
        worst_axis_y = std::max(worst_axis_y, std::fabs(a.y));
        float vpar = std::fabs(P.vel.x*a.x + P.vel.z*a.z);
        worst_par = std::max(worst_par, vpar);
        glm::vec3 c = CenterOf(p);
        glm::vec3 d = c - c_prev; d.y = 0; dist += HLen(d);
        // giro em torno do proprio eixo: angulo relativo menos o yaw
        glm::mat4 O = InstanceOrientation(P);
        float dyaw = std::atan2(a_prev.z*a.x - a_prev.x*a.z, a_prev.x*a.x + a_prev.z*a.z); // giro de a_prev para a em torno de +Y
        yaw_total += dyaw;
        spin += std::sqrt(std::max(0.0f, RelAngle(O_prev, O, 0)*RelAngle(O_prev, O, 0) - dyaw*dyaw));
        c_prev = c; O_prev = O; a_prev = a;
        if (P.body.motion == MOTION_REST) { stopped = true; break; }
        if (!InstFinite(P)) { CHECK(false, "NaN"); break; }
    }
    const Instance& P = g_Instances[p];
    glm::vec3 c = CenterOf(p);
    float R = 1.137f * s;
    printf("   parou=%d em %.2f s; distancia %.3f m; yaw %.3f rad (dist/R_arco %.3f); giro no eixo %.3f (dist/r %.3f)\n",
           stopped, t, dist, yaw_total, dist / R, spin, dist / r);
    printf("   centro (%.3f, %.3f, %.3f); |v.eixo| max %.2e; variacao de altura da AABB %.2e; |eixo.y| max %.2e\n",
           c.x, c.y, c.z, worst_par, worst_h, worst_axis_y);
    CHECK(stopped && P.vel.x == 0 && P.vel.z == 0, "deveria parar");
    CHECK(worst_par < 1e-4f, "velocidade com componente ao longo do eixo (%.2e)", worst_par);
    CHECK(yaw_total > 0.3f, "deveria fazer arco (yaw %.3f) para o lado da cabeca (+)", yaw_total);
    CHECK(std::fabs(yaw_total - dist / R) < 0.05f * (dist / R), "yaw %.3f deveria ser ~ dist/R %.3f", yaw_total, dist / R);
    CHECK(std::fabs(P.yaw - yaw0 - yaw_total) < 1e-3f, "inst.yaw deveria guardar o arco");
    CHECK(std::fabs(spin - dist / r) < 0.03f * (dist / r), "giro no eixo %.3f deveria ser ~ dist/r %.3f", spin, dist / r);
    CHECK(c.x > c0.x + 0.05f, "o arco deveria curvar para o lado da cabeca (+X): x=%.3f", c.x);
    CHECK(worst_h < 1e-4f, "AABB mudou de altura (%.2e)", worst_h);
    CHECK(worst_axis_y < 1e-4f, "eixo saiu da horizontal (%.2e)", worst_axis_y);

    // peao rolando contra a parede: nao atravessa, volta. Parede +Z perto,
    // para a batida acontecer antes de o arco desviar muito.
    NewScene(true, 2.0f, 0.8f);                      // paredes em x = +-2, z = +-0.8
    p = Add(Obj("the_pawn", SHAPE_PAWN, s, glm::vec3(-0.5f, r, 0.3f)));
    LayPawnAlongX(g_Instances[p]);
    g_Instances[p].vel = glm::vec3(0, 0, 2.5f);
    g_Instances[p].body.motion = MOTION_ROLL;
    float worst_pen = 0, vz_before = 0, vz_after = 0; bool touched = false, back = false;
    for (int i = 0; i < (int)(6.0f / DT); ++i)
    {
        float vz_prev = g_Instances[p].vel.z;
        Step(DT);
        AABB bb = BoxOf(p);
        worst_pen = std::max(worst_pen, std::max(std::max(bb.max.x - 2.0f, -2.0f - bb.min.x),
                                                 std::max(bb.max.z - 0.8f, -0.8f - bb.min.z)));
        const Instance& Q = g_Instances[p];
        if (!touched && bb.max.z > 0.8f - 1e-4f) { touched = true; vz_before = vz_prev; vz_after = Q.vel.z; }
        if (touched && bb.max.z < 0.8f - 0.02f) back = true;
        if (!InstFinite(Q)) { CHECK(false, "NaN"); break; }
    }
    printf("   contra parede: encostou=%d (vz %.3f -> %.3f), voltou=%d, penetracao max %.2e, estado %d\n",
           touched, vz_before, vz_after, back, worst_pen, g_Instances[p].body.motion);
    CHECK(touched && vz_before > 0 && vz_after < 0, "peao deveria quicar na parede +Z");
    CHECK(back, "peao deveria se afastar da parede");
    CHECK(worst_pen < 1e-5f, "peao entrou na parede (%.2e)", worst_pen);
    CHECK(g_Instances[p].body.motion == MOTION_REST, "peao deveria parar");

    // peao deitado na borda de uma caixa, pendurado de lado: rola para fora
    NewScene(false);
    Add(SBox(glm::vec3(0, 0.5f, 0), glm::vec3(1.0f)));
    p = Add(Obj("the_pawn", SHAPE_PAWN, s, glm::vec3(0, 1.0f + r, 0.52f)));
    LayPawnAlongX(g_Instances[p]);
    bool fell = false;
    for (int i = 0; i < (int)(8.0f / DT); ++i)
    {
        Step(DT);
        if (g_Instances[p].body.motion == MOTION_FALL) fell = true;
    }
    c = CenterOf(p);
    printf("   pendurado de lado na borda: caiu=%d, final (%.3f, %.3f, %.3f), estado %d\n", fell, c.x, c.y, c.z, g_Instances[p].body.motion);
    CHECK(fell && std::fabs(c.y - r) < 1e-3f && c.z > 0.5f + r, "deveria rolar da caixa para o piso");
    CHECK(g_Instances[p].body.motion == MOTION_REST || g_Instances[p].body.motion == MOTION_ROLL, "estado final");
}

// ---------------------------------------------------------------------------
// e) milhares de passos: sem NaN, orientacao ortonormal
// ---------------------------------------------------------------------------
static void TestLong()
{
    printf("-- e) milhares de passos num cercado\n");
    NewScene(true, 2.0f, 1.5f);
    const float s = 0.56f, r = 0.335f * s;
    int b = Add(Obj("the_sphere", SHAPE_SPHERE, 0.18f, glm::vec3(-1.0f, 0.18f, -0.5f)));
    int p = Add(Obj("the_pawn", SHAPE_PAWN, s, glm::vec3(1.0f, r, 0.3f)));
    int q = Add(Obj("the_sphere", SHAPE_SPHERE, 0.10f, glm::vec3(0.0f, 0.10f, 0.9f)));
    LayPawnAlongX(g_Instances[p]);
    g_Instances[p].yaw = 0.4f;
    g_Instances[b].vel = glm::vec3(3.7f, 0, 1.3f);  g_Instances[b].body.motion = MOTION_ROLL;
    g_Instances[q].vel = glm::vec3(-2.0f, 0, -3.0f); g_Instances[q].body.motion = MOTION_ROLL;
    glm::vec3 a = InstanceLocalAxisWorld(g_Instances[p], 1);
    g_Instances[p].vel = glm::vec3(-a.z, 0, a.x) * 3.0f; g_Instances[p].body.motion = MOTION_ROLL;

    int ids[3] = { b, p, q }; float worst_o = 0, worst_det = 0; bool nan = false; float worst_out = 0;
    int steps = 6000;
    for (int i = 0; i < steps; ++i)
    {
        Step(DT);
        for (int j = 0; j < 3; ++j)
        {
            const Instance& I = g_Instances[ids[j]];
            if (!InstFinite(I)) nan = true;
            glm::mat4 O = InstanceOrientation(I);
            worst_o = std::max(worst_o, OrthoErr(O));
            worst_det = std::max(worst_det, std::fabs(Det3(O) - 1.0f));
            AABB bb = BoxOf(ids[j]);
            worst_out = std::max(worst_out, std::max(std::max(bb.max.x - 2.0f, -2.0f - bb.min.x),
                                                     std::max(bb.max.z - 1.5f, -1.5f - bb.min.z)));
            worst_out = std::max(worst_out, -bb.min.y);
        }
    }
    printf("   %d passos: NaN=%d, erro de ortonormalidade max %.2e, |det-1| max %.2e, saiu do cercado %.2e; estados %d %d %d\n",
           steps, nan, worst_o, worst_det, worst_out,
           g_Instances[b].body.motion, g_Instances[p].body.motion, g_Instances[q].body.motion);
    CHECK(!nan, "NaN");
    CHECK(worst_o < 1e-4f && worst_det < 1e-4f, "orientacao deixou de ser ortonormal");
    CHECK(worst_out < 1e-4f, "saiu do cercado / afundou (%.2e)", worst_out);
    for (int j = 0; j < 3; ++j)
        CHECK(g_Instances[ids[j]].body.motion == MOTION_REST, "objeto %d deveria ter parado", j);
}

// ---------------------------------------------------------------------------
// f) cena real (BuildScene): bola e peao soltos sobre a fase, ficam na sala
// ---------------------------------------------------------------------------
static int FindGrabbable(const char* mesh, int nth)
{
    for (size_t i = 0; i < g_Instances.size(); ++i)
        if (g_Instances[i].grabbable && g_Instances[i].mesh == mesh && nth-- == 0) return (int)i;
    return -1;
}
static void TestRealScene()
{
    printf("-- f) cena real\n");
    const float HX = Room::HALF_X, HZ = Room::HALF_Z;

    struct Case { const char* name; glm::vec3 pos; glm::vec3 vel; bool pawn; };
    const Case cases[] = {
        { "bola cai na quina do dado grande",      glm::vec3(-1.97f, 2.0f, -1.7f), glm::vec3(0),              false },
        { "bola solta a 2 m/s contra a parede -X", glm::vec3(-4.5f, 1.2f,  0.8f), glm::vec3(-2.0f, 0, 0.4f), false },
        { "peao deitado solto a 2 m/s",            glm::vec3( 1.0f, 1.0f,  3.0f), glm::vec3(0.3f, 0, 2.0f),  true  },
    };
    for (int ci = 0; ci < 3; ++ci)
    {
        const Case& C = cases[ci];
        BuildScene(g_Instances);
        int die = FindGrabbable("the_die", 2);       // dado de 0.38 vira um bloco de 1.2 m
        g_Instances[die].scale = glm::vec3(1.2f); g_Instances[die].pos = glm::vec3(-2.6f, 0.6f, -1.7f); g_Instances[die].yaw = 0;
        int o = C.pawn ? FindGrabbable("the_pawn", 0) : FindGrabbable("the_sphere", 0);
        Instance& I = g_Instances[o];
        I.pos = C.pos; I.vel = C.vel; I.body.motion = MOTION_FALL;
        if (C.pawn) LayPawnAlongX(I);
        glm::vec3 lmin, lmax; GetMeshLocalBBox(I.mesh, &lmin, &lmax);
        float rr = C.pawn ? 0.335f * I.scale.x : I.scale.x;

        bool nan = false, rolled = false; float worst_out = 0, t_rest = -1, t = 0, worst_o = 0;
        for (int i = 0; i < (int)(15.0f / DT); ++i)
        {
            Step(DT); t += DT;
            const Instance& J = g_Instances[o];
            if (!InstFinite(J)) { nan = true; break; }
            if (J.body.motion == MOTION_ROLL) rolled = true;
            AABB bb = BoxOf(o);
            worst_out = std::max(worst_out, std::max(std::max(bb.max.x - HX, -HX - bb.min.x),
                                                     std::max(std::max(bb.max.z - HZ, -HZ - bb.min.z), -bb.min.y)));
            worst_o = std::max(worst_o, OrthoErr(InstanceOrientation(J)));
            if (t_rest < 0 && rolled && J.body.motion == MOTION_REST) t_rest = t;
            if (t_rest >= 0 && J.body.motion != MOTION_REST) t_rest = -1;
        }
        const Instance& J = g_Instances[o];
        glm::vec3 c = InstanceBodyCenter(J, lmin, lmax);
        printf("   %s: rolou=%d, parou em %.2f s em (%.2f, %.3f, %.2f), fora da sala %.1e, estado %d\n",
               C.name, rolled, t_rest, c.x, c.y, c.z, worst_out, J.body.motion);
        CHECK(!nan, "%s: NaN", C.name);
        CHECK(rolled, "%s: deveria rolar", C.name);
        CHECK(t_rest > 0 && J.body.motion == MOTION_REST, "%s: deveria parar", C.name);
        CHECK(worst_out < 1e-4f, "%s: saiu da sala / afundou (%.2e)", C.name, worst_out);
        CHECK(std::fabs(c.y - rr) < 2e-3f, "%s: deveria terminar no piso (y=%.4f, r=%.4f)", C.name, c.y, rr);
        CHECK(worst_o < 1e-4f, "%s: orientacao nao ortonormal", C.name);
    }
}

static void RunAll(const char* name)
{
    printf("==== %s\n", name);
    TestUnits();
    TestRest();
    TestEdge();
    TestWall();
    TestPawn();
    TestLong();
    TestRealScene();
}

int main()
{
    g_UsePhysics = false;
    RunAll("laco proprio do teste (OwnStep)");
    int own_fail = g_fail;

    const char* own_only = std::getenv("A3_OWN_ONLY");
    if (!(own_only && own_only[0] == '1'))
    {
        g_UsePhysics = true;
        RunAll("physics.cpp (UpdateFallingObjects)");
    }
    printf("\nt_a3: %d ok, %d falhas (laco proprio: %d; physics.cpp: %d)\n",
           g_pass, g_fail, own_fail, g_fail - own_fail);
    return g_fail ? 1 : 0;
}
