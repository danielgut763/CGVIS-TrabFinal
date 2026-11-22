// Testes do b1-step (M8): passo fixo e ausencia de tunelamento na colisao.
// Rodar com:  tools/check.sh test/t_b1.cpp
//
// Parte A: ResolveAABBMovement / FindSupportHeight isolados, com caixas
//          montadas a mao (parede de 0,2 m, porta desenhada de 0,10 m).
// Parte B: UpdateFallingObjects na cena real (BuildScene), com o passo fixo
//          de 1/240 s e com o passo maximo de 0,1 s.
// Parte C: independencia do FPS com o acumulador de main.cpp.
//
// Sai com codigo != 0 se alguma verificacao falhar.
#include <cstdio>
#include <cmath>
#include <vector>
#include "scene.h"
#include "physics.h"
#include "collisions.h"
#include "grab.h"   // GetMeshLocalBBox (vem de test/stub.cpp)

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++g_fail; printf("   FALHOU: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static AABB Box(float x0, float y0, float z0, float x1, float y1, float z1)
{
    AABB b; b.min = glm::vec3(x0, y0, z0); b.max = glm::vec3(x1, y1, z1); return b;
}
static AABB Moved(const AABB& b, const glm::vec3& d) { AABB o = b; o.min += d; o.max += d; return o; }

// ---------------------------------------------------------------------------
// Cena real: helpers.
// ---------------------------------------------------------------------------
static AABB BoxOf(int i)
{
    glm::vec3 a, b; GetMeshLocalBBox(g_Instances[i].mesh, &a, &b);
    return InstanceCollisionBox(i, a, b);
}
static int FirstMesh(const char* n)
{
    for (size_t i = 0; i < g_Instances.size(); ++i) if (g_Instances[i].mesh == n && g_Instances[i].grabbable) return (int)i;
    return -1;
}
static void Put(int i, glm::vec3 p, float s, glm::vec3 v)
{
    Instance& in = g_Instances[i];
    in.pos = p; in.scale = glm::vec3(s); in.yaw = 0.0f; in.tilt = glm::mat4(1.0f);
    in.vel = v; in.body = BodyState();
}
// Jogador longe de tudo (fora da sala, abaixo do piso): entra na lista como
// obstaculo, mas nunca encosta em nada.
static AABB FarPlayer() { return Box(0.0f, -60.0f, 0.0f, 0.6f, -58.2f, 0.6f); }

// ---------------------------------------------------------------------------
// Copia do laco de passo fixo de main.cpp (que nao compila sem janela).
// ---------------------------------------------------------------------------
static float g_acc = 0.0f;
static int FixedFrame(float frame_dt)
{
    const float h = PhysicsConfig::FIXED_STEP;
    g_acc += frame_dt;
    int n = 0;
    while (g_acc >= h && n < PhysicsConfig::MAX_SUBSTEPS)
    {
        UpdateFallingObjects(-1, FarPlayer(), h);
        g_acc -= h;
        ++n;
    }
    if (g_acc >= h) g_acc = 0.0f;
    return n;
}

int main()
{
    const float H  = PhysicsConfig::FIXED_STEP;   // 1/240 s
    const float V  = PhysicsConfig::MAX_FALL_SPEED; // 20 m/s
    const float TOL = 1e-4f;

    // Cenario sintetico: piso [-0.2, 0], parede +X [6.0, 6.2] (0,2 m), parede
    // -X [-6.2, -6.0], parede +Z [4.5, 4.7], placa fina [6.05, 6.15] (0,10 m,
    // a espessura DESENHADA da porta) numa lista separada.
    std::vector<AABB> room;
    room.push_back(Box(-6.2f, -0.2f, -4.7f,  6.2f, 0.0f, 4.7f));   // 0 piso
    room.push_back(Box( 6.0f,  0.0f, -4.5f,  6.2f, 4.2f, 4.5f));   // 1 parede +X
    room.push_back(Box(-6.2f,  0.0f, -4.5f, -6.0f, 4.2f, 4.5f));   // 2 parede -X
    room.push_back(Box(-6.2f,  0.0f,  4.5f,  6.2f, 4.2f, 4.7f));   // 3 parede +Z
    std::vector<AABB> thin;
    thin.push_back(Box(6.05f, 2.2f, -0.75f, 6.15f, 4.2f, 0.75f));  // placa de 0,10 m

    // Caixa de 0,3 m apoiada no piso, centro em x = 5.
    const AABB c0 = Box(4.85f, 0.0f, -0.15f, 5.15f, 0.3f, 0.15f);
    bool hit[3];

    printf("== Parte A: varredura em ResolveAABBMovement / FindSupportHeight\n");

    // A1) 20 m/s em +X com o passo fixo, 1 s inteiro (o dobro do necessario).
    {
        AABB c = c0; float worst = -1e9f; bool any_hit = false;
        for (int k = 0; k < 240; ++k)
        {
            glm::vec3 a = ResolveAABBMovement(c, glm::vec3(V * H, 0, 0), room, -1, hit);
            c = Moved(c, a); any_hit = any_hit || hit[0];
            if (c.max.x > worst) worst = c.max.x;
        }
        printf("A1 20 m/s, h=1/240, parede 0,2 m: max.x maximo=%.5f (face em 6.0) hit=%d\n", worst, any_hit);
        CHECK(worst <= 6.0f + TOL && std::fabs(c.max.x - 6.0f) < TOL && any_hit, "A1");
    }
    // A2) Mesmo choque em UM passo de 0,1 s (2 m) e em um passo absurdo (100 m).
    {
        glm::vec3 a = ResolveAABBMovement(c0, glm::vec3(V * 0.1f, 0, 0), room, -1, hit);
        AABB c = Moved(c0, a);
        printf("A2 20 m/s, dt=0,1 s (2 m num passo): max.x=%.5f hit=%d", c.max.x, hit[0]);
        CHECK(std::fabs(c.max.x - 6.0f) < TOL && hit[0], "A2 dt=0,1");
        a = ResolveAABBMovement(c0, glm::vec3(100.0f, 0, 0), room, -1, hit);
        c = Moved(c0, a);
        printf(" | 100 m num passo: max.x=%.5f\n", c.max.x);
        CHECK(std::fabs(c.max.x - 6.0f) < TOL, "A2 100 m");
    }
    // A3) Placa de 0,10 m (porta desenhada), na altura dela, 2 m num passo.
    {
        AABB c = Box(4.85f, 3.0f, -0.15f, 5.15f, 3.3f, 0.15f);
        glm::vec3 a = ResolveAABBMovement(c, glm::vec3(V * 0.1f, 0, 0), thin, -1, hit);
        c = Moved(c, a);
        printf("A3 placa de 0,10 m, dt=0,1 s: max.x=%.5f (face em 6.05) hit=%d\n", c.max.x, hit[0]);
        CHECK(std::fabs(c.max.x - 6.05f) < TOL && hit[0], "A3");
    }
    // A4) Sentido negativo (-X), 2 m num passo.
    {
        AABB c = Box(-5.15f, 0.0f, -0.15f, -4.85f, 0.3f, 0.15f);
        glm::vec3 a = ResolveAABBMovement(c, glm::vec3(-V * 0.1f, 0, 0), room, -1, hit);
        c = Moved(c, a);
        printf("A4 -X, dt=0,1 s: min.x=%.5f (face em -6.0) hit=%d\n", c.min.x, hit[0]);
        CHECK(std::fabs(c.min.x + 6.0f) < TOL && hit[0], "A4");
    }
    // A5) Diagonal para o canto (+X,+Z), 20 m/s em cada eixo, dt = 0,1 s.
    {
        AABB c = Box(4.85f, 0.0f, 3.85f, 5.15f, 0.3f, 4.15f);
        glm::vec3 a = ResolveAABBMovement(c, glm::vec3(2.0f, 0, 2.0f), room, -1, hit);
        c = Moved(c, a);
        printf("A5 diagonal no canto: max=(%.5f, %.5f) hit x=%d z=%d\n", c.max.x, c.max.z, hit[0], hit[2]);
        CHECK(std::fabs(c.max.x - 6.0f) < TOL && std::fabs(c.max.z - 4.5f) < TOL, "A5");
    }
    // A6) Vertical: caixa caindo 2 m num passo sobre o piso de 0,2 m.
    {
        AABB c = Moved(c0, glm::vec3(0, 1.0f, 0));
        glm::vec3 a = ResolveAABBMovement(c, glm::vec3(0, -2.0f, 0), room, -1, hit);
        c = Moved(c, a);
        printf("A6 queda de 2 m num passo sobre piso de 0,2 m: min.y=%.5f hit=%d\n", c.min.y, hit[1]);
        CHECK(std::fabs(c.min.y) < TOL && hit[1], "A6");
    }
    // A7) Rolamento a 4 m/s com o passo fixo, 3 s, contra a parede.
    {
        AABB c = c0; float worst = -1e9f;
        for (int k = 0; k < 720; ++k)
        {
            c = Moved(c, ResolveAABBMovement(c, glm::vec3(4.0f * H, 0, 0), room, -1, hit));
            if (c.max.x > worst) worst = c.max.x;
        }
        printf("A7 4 m/s, h=1/240, 3 s: max.x maximo=%.5f\n", worst);
        CHECK(worst <= 6.0f + TOL, "A7");
    }
    // A8) Comecando DENTRO da parede (5 mm, como depois de um giro).
    {
        AABB in = Box(5.705f, 0.0f, -0.15f, 6.005f, 0.3f, 0.15f);
        glm::vec3 a = ResolveAABBMovement(in, glm::vec3(0.05f, 0, 0), room, -1, hit);
        printf("A8 5 mm dentro da parede: +X anda %.4f (esperado 0, hit=%d)", a.x, hit[0]);
        CHECK(a.x == 0.0f && hit[0], "A8 nao pode aprofundar");
        a = ResolveAABBMovement(in, glm::vec3(-0.05f, 0, 0), room, -1, hit);
        printf(" | -X anda %.4f (esperado -0.05)\n", a.x);
        CHECK(std::fabs(a.x + 0.05f) < 1e-6f, "A8 tem que poder sair");

        // Caixa maior que a parede, montada nela: sai para o lado do centro.
        AABB big_l = Box(5.85f, 0.0f, -0.2f, 6.25f, 0.4f, 0.2f);   // centro 6.05 < 6.10
        AABB big_r = Box(5.95f, 0.0f, -0.2f, 6.35f, 0.4f, 0.2f);   // centro 6.15 > 6.10
        float l_p = ResolveAABBMovement(big_l, glm::vec3( 0.05f, 0, 0), room, -1, hit).x;
        float l_m = ResolveAABBMovement(big_l, glm::vec3(-0.05f, 0, 0), room, -1, hit).x;
        float r_p = ResolveAABBMovement(big_r, glm::vec3( 0.05f, 0, 0), room, -1, hit).x;
        float r_m = ResolveAABBMovement(big_r, glm::vec3(-0.05f, 0, 0), room, -1, hit).x;
        printf("A8 montada na parede: centro a -X: +X=%.3f -X=%.3f | centro a +X: +X=%.3f -X=%.3f\n", l_p, l_m, r_p, r_m);
        CHECK(l_p == 0.0f && l_m < 0.0f && r_p > 0.0f && r_m == 0.0f, "A8 montada");
    }
    // A9) Regressao: contato face a face e afundado no piso NAO travam.
    {
        float on_floor = ResolveAABBMovement(c0, glm::vec3(-0.5f, 0, 0), room, -1, hit).x;
        AABB sunk = Moved(c0, glm::vec3(0, -0.005f, 0));
        float sunk_x = ResolveAABBMovement(sunk, glm::vec3(-0.5f, 0, 0), room, -1, hit).x;
        AABB at_wall = Box(5.7f, 0.0f, -0.15f, 6.0f, 0.3f, 0.15f);
        float along = ResolveAABBMovement(at_wall, glm::vec3(0, 0, 1.0f), room, -1, hit).z;
        printf("A9 no piso: %.3f | afundado 5 mm no piso: %.3f | encostado na parede, ao longo dela: %.3f (esperado -0.5, -0.5, 1.0)\n",
               on_floor, sunk_x, along);
        CHECK(std::fabs(on_floor + 0.5f) < 1e-6f && std::fabs(sunk_x + 0.5f) < 1e-6f && std::fabs(along - 1.0f) < 1e-6f, "A9");
    }
    // A10) FindSupportHeight acha o topo MAIS ALTO abaixo, a qualquer distancia.
    {
        std::vector<AABB> v = room;
        v.push_back(Box(4.8f, 0.8f, -0.2f, 5.2f, 1.0f, 0.2f));      // 4: laje de 0,2 m
        AABB c = Moved(c0, glm::vec3(0, 3.0f, 0));                 // base em y = 3
        float hgt = -1.0f; int idx = -1;
        bool ok = FindSupportHeight(c, v, -1, PhysicsConfig::SUPPORT_TOL, &hgt, &idx);
        bool stops = ok && c.min.y - V * 0.1f <= hgt;             // 3 - 2 = 1 <= 1: physics.cpp encaixa
        printf("A10 apoio sob queda de 2 m: achou=%d y=%.3f caixa=%d (esperado laje 4 em 1.000), para na laje=%d\n", ok, hgt, idx, stops);
        CHECK(ok && std::fabs(hgt - 1.0f) < 1e-6f && idx == 4 && stops, "A10");
    }
    // A11) AABBOverlapsAnyExcept ignora as DUAS caixas.
    {
        AABB c = Box(5.8f, -0.01f, -0.1f, 6.01f, 0.3f, 0.1f);     // toca piso (0) e parede (1)
        bool r0 = AABBOverlapsAnyExcept(c, room, 0, 1);
        bool r1 = AABBOverlapsAnyExcept(c, room, 0, -1);
        bool r2 = AABBOverlapsAnyExcept(c, room, -1, 1);
        printf("A11 ignora piso e parede: %d (esperado 0) | so piso: %d (1) | so parede: %d (1)\n", r0, r1, r2);
        CHECK(!r0 && r1 && r2, "A11");
    }

    printf("== Parte B: UpdateFallingObjects na cena real\n");
    BuildScene(g_Instances);
    const int d0 = FirstMesh("the_die");
    const int d1 = d0 + 1;
    const AABB door = BoxOf(g_DoorInstance);
    printf("   porta (colisao): x [%.2f, %.2f] y [%.2f, %.2f]\n", door.min.x, door.max.x, door.min.y, door.max.y);

    // B1/B2) Dado de 0,4 m jogado a 20 m/s contra a parede +X (abaixo da
    //        saida, z = -2), no passo fixo e no passo maximo de 0,1 s.
    const float dts[2] = { H, 0.1f };
    for (int t = 0; t < 2; ++t)
    {
        BuildScene(g_Instances);
        Put(d0, glm::vec3(4.0f, 1.2f, -2.0f), 0.4f, glm::vec3(V, 0, 0));
        float worst = -1e9f;
        int n = (int)(2.0f / dts[t] + 0.5f);
        for (int k = 0; k < n; ++k)
        {
            UpdateFallingObjects(-1, FarPlayer(), dts[t]);
            float mx = BoxOf(d0).max.x;
            if (mx > worst) worst = mx;
        }
        AABB b = BoxOf(d0);
        printf("B%d parede, 20 m/s, dt=%.4f: max.x maximo=%.4f (face 6.0), fim min.y=%.4f x=[%.3f,%.3f]\n",
               t + 1, dts[t], worst, b.min.y, b.min.x, b.max.x);
        CHECK(worst <= 6.0f + 1e-3f && b.max.x <= 6.0f + 1e-3f && std::fabs(b.min.y) < 1e-3f, "B%d", t + 1);
    }
    // B3/B4) Mesma coisa contra a PORTA fechada (y = 3,2, z = 0).
    for (int t = 0; t < 2; ++t)
    {
        BuildScene(g_Instances);
        Put(d0, glm::vec3(4.5f, 3.2f, 0.0f), 0.4f, glm::vec3(V, 0, 0));
        float worst = -1e9f;
        int n = (int)(2.0f / dts[t] + 0.5f);
        for (int k = 0; k < n; ++k)
        {
            UpdateFallingObjects(-1, FarPlayer(), dts[t]);
            float mx = BoxOf(d0).max.x;
            if (mx > worst) worst = mx;
        }
        AABB b = BoxOf(d0);
        printf("B%d porta, 20 m/s, dt=%.4f: max.x maximo=%.4f (face 6.0), fim y=%.4f x=[%.3f,%.3f]\n",
               t + 3, dts[t], worst, b.min.y, b.min.x, b.max.x);
        CHECK(worst <= 6.0f + 1e-3f && b.max.x <= 6.0f + 1e-3f && b.min.y > -1e-3f, "B%d", t + 3);
    }
    // B5) Queda a 20 m/s com dt = 0,1 s (2 m por passo) sobre um dado de 0,3 m.
    {
        BuildScene(g_Instances);
        Put(d1, glm::vec3(-3.0f, 0.15f, 0.0f), 0.3f, glm::vec3(0.0f));
        Put(d0, glm::vec3(-3.0f, 4.0f, 0.0f), 0.4f, glm::vec3(0, -V, 0));
        float lowest = 1e9f;
        for (int k = 0; k < 20; ++k)
        {
            UpdateFallingObjects(-1, FarPlayer(), 0.1f);
            float y = BoxOf(d0).min.y;
            if (y < lowest) lowest = y;
        }
        printf("B5 queda de 2 m/passo sobre dado de 0,3 m: menor min.y=%.4f fim=%.4f (topo em 0.300)\n", lowest, BoxOf(d0).min.y);
        CHECK(lowest > 0.3f - PhysicsConfig::SUPPORT_TOL && std::fabs(BoxOf(d0).min.y - 0.3f) < 1e-3f, "B5");
    }
    // B6) Peao DEITADO (Y local ao longo de X do mundo) a 4 m/s contra a
    //     parede -Z. No M7 o atrito o para na hora; com roll.cpp ele rola ate
    //     a parede. Nos dois casos nao pode atravessa-la.
    {
        BuildScene(g_Instances);
        const int pw = FirstMesh("the_pawn");
        Put(pw, glm::vec3(-3.0f, 0.0f, -3.0f), 0.56f, glm::vec3(0, 0, -4.0f));
        // tilt = rotacao de -90 graus em torno de Z: Y local -> +X do mundo.
        glm::mat4 R(1.0f);
        R[0][0] = 0.0f; R[0][1] = -1.0f;   // coluna 0 = imagem de X local = (0,-1,0)
        R[1][0] = 1.0f; R[1][1] =  0.0f;   // coluna 1 = imagem de Y local = (1, 0,0)
        g_Instances[pw].tilt  = R;
        g_Instances[pw].shape = SHAPE_PAWN;
        g_Instances[pw].body.motion = MOTION_ROLL;
        // Apoia a caixa de colisao no piso.
        g_Instances[pw].pos.y -= BoxOf(pw).min.y;
        float worst = 1e9f;
        for (int k = 0; k < 3 * 240; ++k)
        {
            UpdateFallingObjects(-1, FarPlayer(), H);
            float mz = BoxOf(pw).min.z;
            if (mz < worst) worst = mz;
        }
        AABB b = BoxOf(pw);
        printf("B6 peao deitado a 4 m/s contra a parede -Z: min.z minimo=%.4f (face -4.5), fim z=[%.3f,%.3f] estado=%d\n",
               worst, b.min.z, b.max.z, g_Instances[pw].body.motion);
        CHECK(worst >= -4.5f - 1e-3f, "B6");
    }

    printf("== Parte C: independencia do FPS (acumulador de main.cpp)\n");
    {
        // Dado de 0,4 m jogado na horizontal a 3 m/s de 1,5 m de altura. Ele
        // para quando pousa (atrito), entao o x final depende de EM QUE PASSO
        // ele pousou: com dt variavel, depende do FPS.
        const float fps[3] = { 30.0f, 144.0f, 0.0f };   // 0 = quadros irregulares
        float xf_fixed[3], xf_var[3];
        for (int mode = 0; mode < 2; ++mode)
        {
            for (int f = 0; f < 3; ++f)
            {
                BuildScene(g_Instances);
                Put(d0, glm::vec3(-3.0f, 1.5f, -3.0f), 0.4f, glm::vec3(3.0f, 0, 0));
                g_acc = 0.0f;
                float t = 0.0f; int k = 0; int max_sub = 0;
                while (t < 1.5f)
                {
                    // Irregular: alterna 7, 23, 11 e 40 ms.
                    static const float irr[4] = { 0.007f, 0.023f, 0.011f, 0.040f };
                    float dt = fps[f] > 0.0f ? 1.0f / fps[f] : irr[k % 4];
                    if (mode == 0) { int n = FixedFrame(dt); if (n > max_sub) max_sub = n; }
                    else UpdateFallingObjects(-1, FarPlayer(), dt);
                    t += dt; ++k;
                }
                (mode == 0 ? xf_fixed : xf_var)[f] = g_Instances[d0].pos.x;
                if (mode == 0 && f == 2) printf("   (irregular: no maximo %d passos fixos num quadro)\n", max_sub);
            }
        }
        float spread_fixed = std::fmax(std::fabs(xf_fixed[0] - xf_fixed[1]), std::fabs(xf_fixed[0] - xf_fixed[2]));
        float spread_var   = std::fmax(std::fabs(xf_var[0] - xf_var[1]),     std::fabs(xf_var[0] - xf_var[2]));
        printf("C1 x final, passo fixo:     30 fps %.5f | 144 fps %.5f | irregular %.5f  (diferenca %.6f)\n",
               xf_fixed[0], xf_fixed[1], xf_fixed[2], spread_fixed);
        printf("C1 x final, dt por quadro:  30 fps %.5f | 144 fps %.5f | irregular %.5f  (diferenca %.6f)\n",
               xf_var[0], xf_var[1], xf_var[2], spread_var);
        CHECK(spread_fixed < 1e-4f, "C1 passo fixo deveria dar o mesmo x em qualquer FPS");
    }
    {
        // C2) Teto de sub-passos. Um quadro de 0,1 s (o maximo que main.cpp
        //     manda) cabe em 24 passos. Um quadro maior (so aconteceria se
        //     FIXED_STEP ou MAX_SUBSTEPS mudassem) para no teto e o resto e
        //     descartado, sem virar divida para o quadro seguinte.
        BuildScene(g_Instances);
        g_acc = 0.0f;
        int n1 = FixedFrame(0.1f);
        g_acc = 0.0f;
        int n2 = FixedFrame(0.25f);
        float left = g_acc;
        int n3 = FixedFrame(0.0f);
        printf("C2 quadro de 0,1 s: %d passos | quadro de 0,25 s: %d passos, sobra %.6f | quadro seguinte vazio: %d passos\n",
               n1, n2, left, n3);
        CHECK(n1 >= PhysicsConfig::MAX_SUBSTEPS - 1 && n1 <= PhysicsConfig::MAX_SUBSTEPS &&
              n2 == PhysicsConfig::MAX_SUBSTEPS && left == 0.0f && n3 == 0, "C2");
    }

    if (g_fail) printf("== %d verificacao(oes) FALHARAM\n", g_fail);
    else        printf("== t_b1: tudo ok\n");
    return g_fail ? 1 : 0;
}
