// Testes do a1-orient (rigid.cpp, InstanceCollisionBox, grab com orientacao).
// Rodar: tools/check.sh test/t_a1.cpp
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include "scene.h"
#include "rigid.h"
#include "physics.h"
#include "level.h"
#include "player.h"
#include "grab.h"
#include "matrices.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { if (cond) ++g_pass; else { ++g_fail; printf("FALHOU %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static float MatDiff(const glm::mat4& A, const glm::mat4& B)
{
    float m = 0; for (int c = 0; c < 4; ++c) for (int r = 0; r < 4; ++r) m = std::max(m, std::fabs(A[c][r] - B[c][r])); return m;
}
static float VecDiff(const glm::vec3& a, const glm::vec3& b)
{
    return std::max(std::fabs(a.x-b.x), std::max(std::fabs(a.y-b.y), std::fabs(a.z-b.z)));
}
static float BoxDiff(const AABB& a, const AABB& b) { return std::max(VecDiff(a.min, b.min), VecDiff(a.max, b.max)); }
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
static float Rnd(float a, float b) { return a + (b - a) * (float)rand() / (float)RAND_MAX; }
static glm::vec3 RndAxis() { glm::vec3 v; float n; do { v = glm::vec3(Rnd(-1,1),Rnd(-1,1),Rnd(-1,1)); n = std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z); } while (n < 0.1f || n > 1.0f); return v*(1.0f/n); }
static glm::vec3 Mul3(const glm::mat4& M, const glm::vec3& v) { glm::vec4 r = M*glm::vec4(v.x,v.y,v.z,0); return glm::vec3(r.x,r.y,r.z); }

static Instance MakeObj(const char* mesh, int shape, float s)
{
    Instance i;
    i.mesh = mesh; i.pos = glm::vec3(0.3f, 1.0f, -0.2f); i.yaw = 0.0f; i.scale = glm::vec3(s);
    i.mat = MakeMaterial(0, UV_OBJ, 1.0f); i.solid = true; i.grabbable = true; i.casts_shadow = true;
    i.shape = shape;
    return i;
}

static void TestRotation()
{
    // regra da mao direita: Rz(+90) leva X em Y; Rx(+90) leva Y em Z; Ry(+90) leva Z em X
    glm::mat4 Rz = RotationAboutAxis(glm::vec3(0,0,1), 0.5f*3.14159265f);
    CHECK(VecDiff(Mul3(Rz, glm::vec3(1,0,0)), glm::vec3(0,1,0)) < 1e-6f, "Rz(90)*X != Y");
    glm::mat4 Rx = RotationAboutAxis(glm::vec3(1,0,0), 0.5f*3.14159265f);
    CHECK(VecDiff(Mul3(Rx, glm::vec3(0,1,0)), glm::vec3(0,0,1)) < 1e-6f, "Rx(90)*Y != Z");
    glm::mat4 Ry = RotationAboutAxis(glm::vec3(0,1,0), 0.5f*3.14159265f);
    CHECK(VecDiff(Mul3(Ry, glm::vec3(0,0,1)), glm::vec3(1,0,0)) < 1e-6f, "Ry(90)*Z != X");
    // bate com Matrix_Rotate_X/Y/Z e Matrix_Rotate (matrices.h)
    float maxd = 0, maxo = 0;
    for (int k = 0; k < 200; ++k)
    {
        float a = Rnd(-7, 7);
        maxd = std::max(maxd, MatDiff(RotationAboutAxis(glm::vec3(1,0,0), a), Matrix_Rotate_X(a)));
        maxd = std::max(maxd, MatDiff(RotationAboutAxis(glm::vec3(0,1,0), a), Matrix_Rotate_Y(a)));
        maxd = std::max(maxd, MatDiff(RotationAboutAxis(glm::vec3(0,0,1), a), Matrix_Rotate_Z(a)));
        glm::vec3 ax = RndAxis();
        maxd = std::max(maxd, MatDiff(RotationAboutAxis(ax, a), Matrix_Rotate(a, glm::vec4(ax.x,ax.y,ax.z,0))));
        glm::mat4 R = RotationAboutAxis(ax, a);
        maxo = std::max(maxo, OrthoErr(R));
        // o eixo fica parado
        maxd = std::max(maxd, VecDiff(Mul3(R, ax), ax));
        // eixo nao normalizado e tratado como unitario
        maxd = std::max(maxd, MatDiff(RotationAboutAxis(ax*3.0f, a), R));
    }
    CHECK(maxd < 2e-5f, "RotationAboutAxis difere de Matrix_Rotate*: %g", maxd);
    CHECK(maxo < 2e-6f, "RotationAboutAxis nao ortonormal: %g", maxo);
    CHECK(MatDiff(RotationAboutAxis(glm::vec3(0), 1.0f), glm::mat4(1.0f)) == 0.0f, "eixo nulo deveria dar identidade");

    // Ortonormalizacao: 100000 rotacoes pequenas compostas sem corrigir saem
    // da ortonormalidade; corrigindo, ficam.
    glm::mat4 A(1.0f), B(1.0f);
    for (int k = 0; k < 100000; ++k)
    {
        glm::mat4 d = RotationAboutAxis(glm::vec3(0.3f, 0.8f, -0.52f), 0.0123f);
        A = d * A;
        B = d * B; OrthonormalizeRotation(B);
    }
    printf("  ortonormalidade apos 1e5 passos: sem %.2e, com %.2e (det %.7f)\n", OrthoErr(A), OrthoErr(B), Det3(B));
    CHECK(OrthoErr(B) < 1e-6f && std::fabs(Det3(B) - 1.0f) < 1e-5f, "OrthonormalizeRotation falhou: %g", OrthoErr(B));
    // e nao muda uma rotacao que ja e ortonormal (alem do arredondamento)
    glm::mat4 C = RotationAboutAxis(glm::vec3(0.2f, -0.4f, 0.9f), 2.1f), C2 = C; OrthonormalizeRotation(C2);
    CHECK(MatDiff(C, C2) < 1e-6f, "OrthonormalizeRotation mexeu numa rotacao valida: %g", MatDiff(C, C2));
}

static void TestApplyAndCenters()
{
    glm::vec3 lmin(-1.0f, -0.991f, -0.775f), lmax(1.2f, 1.1f, 0.6f); // bbox fora do centro de proposito
    Instance a = MakeObj("x", SHAPE_BOX, 0.4f);
    a.yaw = 0.7f;
    a.tilt = RotationAboutAxis(glm::vec3(1, 0, 0), 0.3f);
    glm::mat4 O0 = InstanceOrientation(a);
    glm::vec3 ax = glm::vec3(0.3f, -0.2f, 0.93f); ax = ax * (1.0f/std::sqrt(0.09f+0.04f+0.93f*0.93f));
    ApplyWorldRotation(a, ax, 0.9f);
    CHECK(a.yaw == 0.7f, "ApplyWorldRotation mudou o yaw");
    CHECK(MatDiff(InstanceOrientation(a), RotationAboutAxis(ax, 0.9f) * O0) < 1e-5f, "ApplyWorldRotation: O' != R*O");
    CHECK(a.tilt[3][0] == 0 && a.tilt[3][1] == 0 && a.tilt[3][2] == 0 && a.tilt[3][3] == 1, "tilt com 4a coluna errada");

    // centro: ida e volta
    glm::vec3 c = InstanceBodyCenter(a, lmin, lmax);
    glm::vec4 cm = InstanceModelMatrix(a) * glm::vec4(0.1f, 0.0545f, -0.0875f, 1.0f);
    CHECK(VecDiff(c, glm::vec3(cm.x, cm.y, cm.z)) < 1e-6f, "InstanceBodyCenter != Model*c_local");
    SetInstanceBodyCenter(a, lmin, lmax, glm::vec3(2, 3, 4));
    CHECK(VecDiff(InstanceBodyCenter(a, lmin, lmax), glm::vec3(2, 3, 4)) < 1e-5f, "SetInstanceBodyCenter ida e volta");

    // girar em torno do centro: centro fica
    RotateAboutCenter(a, lmin, lmax, glm::vec3(0, 0, 1), 1.3f);
    CHECK(VecDiff(InstanceBodyCenter(a, lmin, lmax), glm::vec3(2, 3, 4)) < 1e-5f, "RotateAboutCenter moveu o centro");

    // girar em torno de uma reta: todo ponto do corpo vai para pivot + R*(p - pivot)
    glm::vec3 pivot(2.5f, 2.6f, 3.7f), ln(0, 0, 1);
    glm::mat4 M0 = InstanceModelMatrix(a);
    RotateAboutLine(a, lmin, lmax, pivot, ln, -0.6f);
    glm::mat4 R = RotationAboutAxis(ln, -0.6f);
    float md = 0;
    for (int k = 0; k < 8; ++k)
    {
        glm::vec4 p(k&1 ? lmax.x : lmin.x, k&2 ? lmax.y : lmin.y, k&4 ? lmax.z : lmin.z, 1.0f);
        glm::vec4 p0 = M0 * p, p1 = InstanceModelMatrix(a) * p;
        glm::vec3 q0(p0.x, p0.y, p0.z), q1(p1.x, p1.y, p1.z);
        md = std::max(md, VecDiff(q1, pivot + Mul3(R, q0 - pivot)));
    }
    CHECK(md < 1e-5f, "RotateAboutLine: cantos fora do arco (%g)", md);
    // um tombo de 90 graus em torno da aresta x=+0.5 de um cubo de lado 1 no chao
    Instance cube = MakeObj("the_cube", SHAPE_BOX, 1.0f);
    cube.pos = glm::vec3(0, 0.5f, 0);
    RotateAboutLine(cube, glm::vec3(-0.5f), glm::vec3(0.5f), glm::vec3(0.5f, 0, 0), glm::vec3(0, 0, -1), 0.5f*3.14159265f);
    CHECK(VecDiff(cube.pos, glm::vec3(1.0f, 0.5f, 0)) < 1e-5f, "tombo de 90 graus: centro (%.3f %.3f %.3f)", cube.pos.x, cube.pos.y, cube.pos.z);
}

static void TestStablePose()
{
    const float PI = 3.14159265f;
    glm::vec3 ax; float ang;
    // BOX: qualquer orientacao, depois da rotacao sugerida, fica estavel
    float worst = 0;
    for (int k = 0; k < 500; ++k)
    {
        Instance b = MakeObj("the_die", SHAPE_BOX, 0.3f);
        b.yaw = Rnd(-4, 4); b.tilt = RotationAboutAxis(RndAxis(), Rnd(0, PI));
        bool moved = NearestStablePose(b, 0.0f, &ax, &ang);
        if (!moved) continue;
        worst = std::max(worst, ang);
        ApplyWorldRotation(b, ax, ang);
        CHECK(!NearestStablePose(b, 1e-3f, &ax, &ang), "BOX nao ficou estavel (resto %g)", ang);
    }
    printf("  BOX: maior rotacao ate a pose estavel %.1f graus (teorico 54.7)\n", worst*180/PI);
    CHECK(worst < 54.8f*PI/180, "BOX: rotacao grande demais %g", worst);
    // BOX estavel: nada a fazer
    {
        Instance b = MakeObj("the_die", SHAPE_BOX, 0.3f); b.yaw = 1.0f;
        b.tilt = RotationAboutAxis(glm::vec3(1, 0, 0), PI*0.5f); // de lado: face +Z para cima
        CHECK(!NearestStablePose(b, 0.01f, &ax, &ang) && ang == 0.0f, "BOX de lado deveria ser estavel");
        b.tilt = RotationAboutAxis(glm::vec3(1, 0, 0), PI*0.5f + 0.2f);
        CHECK(NearestStablePose(b, 0.01f, &ax, &ang) && std::fabs(ang - 0.2f) < 1e-4f, "BOX 0.2 rad fora: angulo %g", ang);
    }
    // PAWN
    {
        Instance p = MakeObj("the_pawn", SHAPE_PAWN, 0.56f); p.yaw = 0.4f;
        p.tilt = RotationAboutAxis(glm::vec3(1, 0, 0), 30*PI/180);
        CHECK(NearestStablePose(p, 0.01f, &ax, &ang) && std::fabs(ang - 30*PI/180) < 1e-4f, "PAWN 30 graus deveria levantar (%g)", ang);
        ApplyWorldRotation(p, ax, ang);
        CHECK(InstanceLocalAxisWorld(p, 1).y > 0.99999f, "PAWN nao levantou");
        p.tilt = RotationAboutAxis(glm::vec3(1, 0, 0), 60*PI/180);
        CHECK(NearestStablePose(p, 0.01f, &ax, &ang) && std::fabs(ang - 30*PI/180) < 1e-4f, "PAWN 60 graus deveria deitar girando 30 (%g)", ang);
        ApplyWorldRotation(p, ax, ang);
        CHECK(std::fabs(InstanceLocalAxisWorld(p, 1).y) < 1e-5f && PawnIsLying(p), "PAWN nao deitou");
        CHECK(!NearestStablePose(p, 0.01f, &ax, &ang), "PAWN deitado deveria ser estavel");
        // deitado girando em torno do proprio eixo continua estavel
        RotateAboutCenter(p, glm::vec3(-0.335f, -0.5f, -0.335f), glm::vec3(0.335f, 0.5f, 0.335f), InstanceLocalAxisWorld(p, 1), 1.234f);
        CHECK(!NearestStablePose(p, 0.01f, &ax, &ang), "PAWN deitado e girado no eixo deveria ser estavel");
        // limite de equilibrio pela geometria: atan(r_base/h_cm) = atan(0.335/0.5) ~ 33.8 graus
        float lim = std::atan(0.335f/0.5f);
        printf("  PAWN: limite de equilibrio em pe %.1f graus\n", lim*180/PI);
        p.tilt = RotationAboutAxis(glm::vec3(1, 0, 0), lim - 0.02f);
        CHECK(NearestStablePose(p, 0.01f, &ax, &ang) && std::fabs(ang - (lim - 0.02f)) < 1e-4f, "PAWN logo abaixo do limite deveria levantar (%g)", ang);
        p.tilt = RotationAboutAxis(glm::vec3(1, 0, 0), lim + 0.02f);
        CHECK(NearestStablePose(p, 0.01f, &ax, &ang) && std::fabs(ang - (PI*0.5f - lim - 0.02f)) < 1e-4f, "PAWN logo acima do limite deveria deitar (%g)", ang);
        p.tilt = RotationAboutAxis(glm::vec3(1, 0, 0), 40*PI/180); // antes (45 graus) levantava
        CHECK(NearestStablePose(p, 0.01f, &ax, &ang) && std::fabs(ang - 50*PI/180) < 1e-4f, "PAWN a 40 graus deveria deitar (%g)", ang);
        // de cabeca para baixo: deita
        p.tilt = RotationAboutAxis(glm::vec3(0, 0, 1), PI);
        CHECK(NearestStablePose(p, 0.01f, &ax, &ang) && std::fabs(ang - PI*0.5f) < 1e-4f, "PAWN de cabeca para baixo deveria deitar (%g)", ang);
        ApplyWorldRotation(p, ax, ang);
        CHECK(PawnIsLying(p) && std::fabs(InstanceLocalAxisWorld(p, 1).y) < 1e-5f, "PAWN invertido nao deitou");
        p.tilt = RotationAboutAxis(glm::vec3(0, 0, 1), PI - 0.3f); // quase invertido
        CHECK(NearestStablePose(p, 0.01f, &ax, &ang) && std::fabs(ang - (PI*0.5f - 0.3f)) < 1e-4f, "PAWN quase invertido (%g)", ang);
    }
    // SPHERE: sempre estavel
    {
        Instance s = MakeObj("the_sphere", SHAPE_SPHERE, 0.18f);
        s.tilt = RotationAboutAxis(RndAxis(), 2.0f);
        CHECK(!NearestStablePose(s, 0.0f, &ax, &ang) && ang == 0.0f, "SPHERE deveria ser sempre estavel");
    }
}

static bool IsCubeRotation(const glm::mat4& T)
{
    for (int c = 0; c < 3; ++c) { int nz = 0; for (int r = 0; r < 3; ++r) { float v = T[c][r]; if (v != 0.0f && v != 1.0f && v != -1.0f) return false; if (v != 0.0f) ++nz; } if (nz != 1) return false; }
    return Det3(T) == 1.0f && T[3][3] == 1.0f && T[3][0] == 0 && T[3][1] == 0 && T[3][2] == 0;
}

static void TestCanonical()
{
    const float PI = 3.14159265f;
    // as 24 rotacoes do cubo: compostas de Rx/Ry/Rz de 90 graus
    float worst = 0; int bad = 0;
    for (int k = 0; k < 600; ++k)
    {
        Instance b = MakeObj("the_cube", SHAPE_BOX, Rnd(0.2f, 2.0f));
        b.yaw = Rnd(-6, 6);
        glm::mat4 T(1.0f);
        int n = rand() % 6;
        for (int q = 0; q < n; ++q) { int e = rand() % 3; glm::vec3 a(e == 0, e == 1, e == 2); T = RotationAboutAxis(a, PI*0.5f*(1 + rand()%3)) * T; }
        // mais um giro em torno do Y de MUNDO (que vira yaw) e ruido pequeno
        float extra = Rnd(-3, 3);
        glm::mat4 O = Matrix_Rotate_Y(extra) * Matrix_Rotate_Y(b.yaw) * T;
        O = RotationAboutAxis(RndAxis(), Rnd(0, 2e-4f)) * O;
        b.tilt = Matrix_Rotate_Y(-b.yaw) * O;
        OrthonormalizeRotation(b.tilt);
        glm::mat4 M0 = InstanceModelMatrix(b);
        float yaw0 = b.yaw;
        CanonicalizePose(b);
        float d = MatDiff(InstanceModelMatrix(b), M0);
        worst = std::max(worst, d);
        if (!IsCubeRotation(b.tilt)) ++bad;
        if (std::fabs(b.yaw - yaw0) > PI*0.25f + 1e-3f) ++bad; // yaw continua perto do anterior
    }
    printf("  BOX canonico: maior diferenca na Model %.2e\n", worst);
    CHECK(worst < 1e-3f, "CanonicalizePose BOX mudou a Model %g", worst);
    CHECK(bad == 0, "CanonicalizePose BOX: %d tilts nao sao rotacao do cubo / yaw saltou", bad);
    // sem ruido: < 1e-4
    {
        Instance b = MakeObj("the_cube", SHAPE_BOX, 1.5f);
        b.yaw = 0.37f;
        b.tilt = RotationAboutAxis(glm::vec3(0, 1, 0), 1.1f) * RotationAboutAxis(glm::vec3(1, 0, 0), PI*0.5f) * RotationAboutAxis(glm::vec3(0, 0, 1), PI);
        glm::mat4 M0 = InstanceModelMatrix(b);
        CanonicalizePose(b);
        CHECK(MatDiff(InstanceModelMatrix(b), M0) < 1e-4f && IsCubeRotation(b.tilt), "CanonicalizePose BOX exato: %g", MatDiff(InstanceModelMatrix(b), M0));
    }
    // objeto que nunca tombou: tilt = I e yaw igual
    {
        Instance b = MakeObj("the_die", SHAPE_BOX, 0.34f); b.yaw = 2.9f;
        Instance b0 = b; CanonicalizePose(b);
        CHECK(MatDiff(b.tilt, glm::mat4(1.0f)) == 0.0f && std::fabs(b.yaw - 2.9f) < 1e-5f && VecDiff(b.pos, b0.pos) < 1e-6f, "CanonicalizePose mexeu num objeto em pe");
    }
    // PAWN em pe com tilt = giro em Y: tilt vira I, yaw absorve
    {
        Instance p = MakeObj("the_pawn", SHAPE_PAWN, 0.56f); p.yaw = 0.5f;
        p.tilt = RotationAboutAxis(glm::vec3(0, 1, 0), 0.8f) * RotationAboutAxis(glm::vec3(1, 0, 0), 1e-5f);
        glm::mat4 M0 = InstanceModelMatrix(p);
        CanonicalizePose(p);
        CHECK(MatDiff(p.tilt, glm::mat4(1.0f)) == 0.0f && std::fabs(p.yaw - 1.3f) < 1e-4f && MatDiff(InstanceModelMatrix(p), M0) < 1e-4f,
              "CanonicalizePose PAWN em pe: yaw %g dif %g", p.yaw, MatDiff(InstanceModelMatrix(p), M0));
    }
    // PAWN deitado: so ortonormaliza
    {
        Instance p = MakeObj("the_pawn", SHAPE_PAWN, 0.56f); p.yaw = 0.5f;
        p.tilt = RotationAboutAxis(glm::vec3(0, 0, 1), PI*0.5f) * RotationAboutAxis(glm::vec3(0, 1, 0), 0.7f);
        glm::mat4 M0 = InstanceModelMatrix(p);
        CanonicalizePose(p);
        CHECK(MatDiff(InstanceModelMatrix(p), M0) < 1e-5f && PawnIsLying(p), "CanonicalizePose PAWN deitado mudou a pose");
    }
}

static void TestAABB()
{
    const float PI = 3.14159265f;
    glm::vec3 pmin(-0.335f, -0.5f, -0.335f), pmax = -pmin;
    // PAWN em pe, yaw qualquer: caixa igual a sem yaw (corpo de revolucao)
    Instance p = MakeObj("the_pawn", SHAPE_PAWN, 0.56f);
    AABB up0 = BodyWorldAABB(p, pmin, pmax);
    CHECK(BoxDiff(up0, AABBTransform(pmin, pmax, InstanceModelMatrix(p))) < 1e-6f, "PAWN em pe sem yaw deveria bater com AABBTransform");
    p.yaw = 0.7f;
    CHECK(BoxDiff(BodyWorldAABB(p, pmin, pmax), up0) < 1e-6f, "PAWN em pe: yaw mudou a caixa");
    // PAWN deitado (eixo ao longo de X) girando em torno do proprio eixo: caixa constante
    p.yaw = 0.0f; p.tilt = RotationAboutAxis(glm::vec3(0, 0, 1), -PI*0.5f);
    glm::vec3 c = InstanceBodyCenter(p, pmin, pmax);
    AABB l0 = BodyWorldAABB(p, pmin, pmax);
    float maxd = 0, maxgrow = 0;
    for (int k = 1; k <= 64; ++k)
    {
        RotateAboutCenter(p, pmin, pmax, InstanceLocalAxisWorld(p, 1), 0.1f);
        maxd = std::max(maxd, BoxDiff(BodyWorldAABB(p, pmin, pmax), l0));
        AABB tr = AABBTransform(pmin, pmax, InstanceModelMatrix(p));
        maxgrow = std::max(maxgrow, (tr.max.y - tr.min.y) / (l0.max.y - l0.min.y));
    }
    CHECK(maxd < 1e-5f, "PAWN deitado girando no eixo: caixa variou %g", maxd);
    printf("  PAWN deitado: caixa cilindro constante (dif %.1e); a dos 8 cantos cresceria ate %.0f%%\n", maxd, (maxgrow - 1)*100);
    CHECK(std::fabs((l0.max.x - l0.min.x) - 0.56f) < 1e-5f && std::fabs((l0.max.y - l0.min.y) - 0.67f*0.56f) < 1e-5f, "PAWN deitado: dimensoes");
    CHECK(VecDiff(AABBCenter(l0), c) < 1e-6f, "PAWN: caixa nao centrada no centro");
    // cilindro inclinado 45 graus no plano XY: meia-extensao x = h*s + r*s
    p.tilt = RotationAboutAxis(glm::vec3(0, 0, 1), PI*0.25f); p.yaw = 0;
    AABB t45 = BodyWorldAABB(p, pmin, pmax);
    float s = std::sqrt(0.5f), h = 0.28f, r = 0.335f*0.56f;
    CHECK(std::fabs(0.5f*(t45.max.x - t45.min.x) - (h*s + r*s)) < 1e-5f && std::fabs(0.5f*(t45.max.z - t45.min.z) - r) < 1e-5f, "PAWN 45 graus: meia-extensao");
    // SPHERE: centro +- raio, nao muda com rotacao
    Instance b = MakeObj("the_sphere", SHAPE_SPHERE, 0.18f);
    glm::vec3 smin(-1), smax(1);
    AABB s0 = BodyWorldAABB(b, smin, smax);
    CHECK(std::fabs(s0.max.y - s0.min.y - 0.36f) < 1e-6f, "SPHERE: diametro");
    b.tilt = RotationAboutAxis(RndAxis(), 1.1f);
    CHECK(BoxDiff(BodyWorldAABB(b, smin, smax), s0) < 1e-6f, "SPHERE: rotacao mudou a caixa");
    // BOX: AABBTransform
    Instance d = MakeObj("the_die", SHAPE_BOX, 0.3f); d.tilt = RotationAboutAxis(RndAxis(), 0.8f); d.yaw = 0.3f;
    CHECK(BoxDiff(BodyWorldAABB(d, glm::vec3(-0.5f), glm::vec3(0.5f)), AABBTransform(glm::vec3(-0.5f), glm::vec3(0.5f), InstanceModelMatrix(d))) == 0.0f, "BOX != AABBTransform");
}

static void TestScene()
{
    BuildScene(g_Instances);
    int ball = -1, npawn = 0;
    for (size_t i = 0; i < g_Instances.size(); ++i)
    {
        const Instance& in = g_Instances[i];
        if (in.mesh == "the_sphere" && in.grabbable) ball = (int)i;
        if (in.mesh == "the_pawn") { ++npawn; CHECK(in.shape == SHAPE_PAWN, "peao %d sem SHAPE_PAWN", (int)i); }
        if (in.grabbable && in.mesh != "the_pawn" && in.mesh != "the_sphere") CHECK(in.shape == SHAPE_BOX, "pegavel %d (%s) deveria ser SHAPE_BOX", (int)i, in.mesh.c_str());
    }
    printf("  bola pegavel = instancia %d (Bezier = %d), %d instancias no total\n", ball, g_BezierInstance, (int)g_Instances.size());
    CHECK(npawn == 2, "esperava 2 peoes");
    CHECK(ball > g_BezierInstance && g_BezierInstance >= 0, "bola deveria vir depois da esfera da Bezier");
    CHECK(ball >= 0 && g_Instances[ball].shape == SHAPE_SPHERE && g_Instances[ball].solid && g_Instances[ball].casts_shadow, "bola: flags");
    if (ball < 0) return;
    glm::vec3 a, b; GetMeshLocalBBox("the_sphere", &a, &b);
    AABB bb = InstanceCollisionBox(ball, a, b);
    CHECK(std::fabs(bb.min.y) < 1e-6f && std::fabs(bb.max.y - 0.36f) < 1e-5f, "bola deveria estar apoiada no piso com raio 0.18 (%g %g)", bb.min.y, bb.max.y);
    // longe do spawn e sem encostar em nada solido (exceto o piso, que so toca)
    CHECK(std::fabs(bb.min.x + 4.6f) > 1.5f || std::fabs(bb.min.z) > 1.5f, "bola perto do spawn");
    for (size_t i = 0; i < g_Instances.size(); ++i)
    {
        if ((int)i == ball || !g_Instances[i].solid) continue;
        glm::vec3 la, lb; GetMeshLocalBBox(g_Instances[i].mesh, &la, &lb);
        AABB o = InstanceCollisionBox((int)i, la, lb);
        CHECK(!AABBOverlap(AABBExpand(bb, -1e-3f), AABBExpand(o, 0.3f)) || i == 0, "bola a menos de 30 cm da instancia %d (%s)", (int)i, g_Instances[i].mesh.c_str());
    }
    // a caixa de colisao de um peao com yaw nao incha (cilindro)
    int pw = -1; for (size_t i = 0; i < g_Instances.size(); ++i) if (g_Instances[i].mesh == "the_pawn") { pw = (int)i; break; }
    GetMeshLocalBBox("the_pawn", &a, &b);
    AABB p0 = InstanceCollisionBox(pw, a, b);
    g_Instances[pw].yaw = 0.785f;
    AABB p1 = InstanceCollisionBox(pw, a, b);
    CHECK(BoxDiff(p0, p1) < 1e-6f, "peao com yaw mudou a caixa de colisao");
    // e a porta continua com o override (espessura inteira da parede)
    GetMeshLocalBBox(g_Instances[g_DoorInstance].mesh, &a, &b);
    AABB door = InstanceCollisionBox(g_DoorInstance, a, b);
    CHECK(door.min.x == Room::HALF_X && door.max.x == Room::HALF_X + Room::THICKNESS, "porta perdeu o override");
}

static std::vector<AABB> SolidBoxes(std::vector<int>* owner)
{
    std::vector<AABB> v; owner->clear();
    for (size_t i = 0; i < g_Instances.size(); ++i)
    {
        if (!g_Instances[i].solid) continue;
        glm::vec3 a, b; GetMeshLocalBBox(g_Instances[i].mesh, &a, &b);
        v.push_back(InstanceCollisionBox((int)i, a, b)); owner->push_back((int)i);
    }
    return v;
}

static void TestGrab()
{
    const float PI = 3.14159265f;
    BuildScene(g_Instances);
    int pw = -1; for (size_t i = 0; i < g_Instances.size(); ++i) if (g_Instances[i].mesh == "the_pawn") { pw = (int)i; break; }
    glm::vec3 la, lb; GetMeshLocalBBox("the_pawn", &la, &lb);
    Instance& p = g_Instances[pw];
    // peao deitado (eixo ao longo de +X) no piso, no meio de um "tombo"
    p.tilt = RotationAboutAxis(glm::vec3(0, 0, 1), -PI*0.5f);
    SetInstanceBodyCenter(p, la, lb, glm::vec3(-0.6f, 0.335f*0.56f, -3.0f));
    p.body.motion = MOTION_ROLL; p.body.ang_vel = glm::vec3(0, 0, 3); p.vel = glm::vec3(1, 0, 0);
    glm::mat4 tilt0 = p.tilt;
    AABB far_player; far_player.min = glm::vec3(100); far_player.max = glm::vec3(101);

    Camera cam; glm::vec3 eye(-0.6f, 1.2f, 0.5f);
    glm::vec3 to = InstanceBodyCenter(p, la, lb) - eye;
    CameraInit(cam, eye, -PI*0.5f, std::atan2(to.y, std::sqrt(to.x*to.x + to.z*to.z)));
    GrabState g; GrabInit(g);
    std::vector<int> owner; std::vector<AABB> boxes = SolidBoxes(&owner);
    bool picked = GrabTryPick(g, cam, 16.0f/9.0f, 0.0f, 0.0f, boxes, owner);
    CHECK(picked && g.instance == pw, "nao pegou o peao deitado");
    if (!picked) return;
    CHECK(MatDiff(g_Instances[pw].tilt, tilt0) == 0.0f, "pegar mudou o tilt");
    CHECK(g_Instances[pw].body.motion == MOTION_REST && g_Instances[pw].body.ang_vel == glm::vec3(0) && g_Instances[pw].vel == glm::vec3(0), "pegar nao zerou vel/body");

    // na mao: o centro do corpo fica exatamente na mira e o tilt continua
    const float dt = 1.0f/60.0f;
    boxes = SolidBoxes(&owner);
    GrabUpdate(g, cam, boxes, owner, far_player, dt);
    glm::vec3 fwd(CameraForward(cam).x, CameraForward(cam).y, CameraForward(cam).z);
    glm::vec3 c = InstanceBodyCenter(g_Instances[pw], la, lb);
    CHECK(VecDiff(c, eye + fwd*g.cur_distance) < 1e-4f, "centro fora da mira (%g)", VecDiff(c, eye + fwd*g.cur_distance));
    CHECK(MatDiff(g_Instances[pw].tilt, tilt0) == 0.0f, "GrabUpdate mudou o tilt");
    AABB hb = InstanceCollisionBox(pw, la, lb);
    printf("  peao deitado na mao: d=%.3f fator=%.2f bloqueado=%d caixa y %.3f..%.3f\n", g.cur_distance, g.cur_factor, g.blocked, hb.min.y, hb.max.y);
    CHECK(!AABBOverlapsAny(AABBExpand(hb, -GrabConfig::PLACE_MARGIN), boxes, -1), "objeto na mao penetra a cena");
    CHECK(hb.min.y < 0.02f, "olhando para o piso, o peao deveria encostar no piso (min.y %.4f)", hb.min.y);

    // Q/E: gira em torno do Y do mundo pelo centro; continua deitado
    GrabRotate(g, PI*0.5f);
    glm::vec3 c2 = InstanceBodyCenter(g_Instances[pw], la, lb);
    CHECK(VecDiff(c2, c) < 1e-5f && PawnIsLying(g_Instances[pw]), "GrabRotate moveu o centro ou levantou o peao");
    glm::vec3 u = InstanceLocalAxisWorld(g_Instances[pw], 1);
    CHECK(std::fabs(std::fabs(u.z) - 1.0f) < 1e-5f, "GrabRotate de 90 graus deveria levar o eixo de X para Z (%g %g %g)", u.x, u.y, u.z);
    GrabRotate(g, -PI*0.5f);

    // velocidade herdada: camera andando de lado a 1 m/s por 0.5 s
    for (int k = 0; k < 30; ++k) { cam.eye.x += 1.0f*dt; boxes = SolidBoxes(&owner); GrabUpdate(g, cam, boxes, owner, far_player, dt); }
    printf("  vel media na mao (camera a 1 m/s em +X): (%.3f %.3f %.3f)\n", g.vel_avg.x, g.vel_avg.y, g.vel_avg.z);
    bool rel = GrabTryRelease(g, cam, boxes, owner, far_player);
    CHECK(rel, "nao soltou");
    Instance& q = g_Instances[pw];
    CHECK(VecDiff(q.vel, glm::vec3(1, 0, 0)) < 0.02f, "vel herdada (%g %g %g) != (1,0,0)", q.vel.x, q.vel.y, q.vel.z);
    CHECK(q.body.motion == MOTION_FALL && q.solid && !GrabIsHolding(g), "soltar: estado");
    AABB rb = InstanceCollisionBox(pw, la, lb);
    boxes = SolidBoxes(&owner);
    int self = -1; for (size_t i = 0; i < owner.size(); ++i) if (owner[i] == pw) self = (int)i;
    CHECK(!AABBOverlapsAny(AABBExpand(rb, -GrabConfig::PLACE_MARGIN), boxes, self), "solto dentro da cena");

    // rapido demais: limitado a RELEASE_MAX_SPEED
    boxes = SolidBoxes(&owner);
    cam.pitch = std::atan2(InstanceBodyCenter(q, la, lb).y - cam.eye.y, 3.5f);
    picked = GrabTryPick(g, cam, 16.0f/9.0f, 0.0f, 0.0f, boxes, owner);
    CHECK(picked, "nao pegou de novo");
    if (picked)
    {
        for (int k = 0; k < 30; ++k) { cam.eye.x -= 8.0f*dt; boxes = SolidBoxes(&owner); GrabUpdate(g, cam, boxes, owner, far_player, dt); }
        float sp_hand = std::sqrt(g.vel_avg.x*g.vel_avg.x + g.vel_avg.y*g.vel_avg.y + g.vel_avg.z*g.vel_avg.z);
        GrabTryRelease(g, cam, boxes, owner, far_player);
        float sp = std::sqrt(q.vel.x*q.vel.x + q.vel.y*q.vel.y + q.vel.z*q.vel.z);
        printf("  mao a %.2f m/s -> solto a %.3f m/s (limite %.1f)\n", sp_hand, sp, GrabConfig::RELEASE_MAX_SPEED);
        CHECK(std::fabs(sp - GrabConfig::RELEASE_MAX_SPEED) < 1e-4f && q.vel.x < 0, "nao limitou a velocidade");
    }
    // sem dt (main.cpp antigo): solta parado
    boxes = SolidBoxes(&owner);
    picked = GrabTryPick(g, cam, 16.0f/9.0f, 0.0f, 0.0f, boxes, owner);
    if (picked)
    {
        for (int k = 0; k < 10; ++k) { cam.eye.x += 1.0f*dt; boxes = SolidBoxes(&owner); GrabUpdate(g, cam, boxes, owner, far_player); }
        GrabTryRelease(g, cam, boxes, owner, far_player);
        CHECK(q.vel == glm::vec3(0), "sem dt deveria soltar parado");
    }
    CHECK(picked, "nao pegou pela terceira vez");
    // a fisica (ainda M7 nesta branch) assenta o peao deitado com a caixa cilindrica no piso
    q.vel = glm::vec3(0); q.body = BodyState();
    for (int k = 0; k < 240; ++k) UpdateFallingObjects(-1, far_player, 1.0f/120);
    AABB fb = InstanceCollisionBox(pw, la, lb);
    printf("  peao deitado depois de 2 s: caixa y %.4f..%.4f (esperado 0..%.4f), deitado=%d\n", fb.min.y, fb.max.y, 2*0.335f*q.scale.x, PawnIsLying(q));
    CHECK(std::fabs(fb.min.y) < 0.021f && PawnIsLying(q), "peao deitado nao assentou no piso");
}

// Muitas miras aleatorias com objetos tombados de todas as formas: quando o
// grab diz que cabe (!blocked), a caixa de COLISAO (a da fisica) nao entra na
// cena, e o centro do corpo esta na mira.
static void TestGrabRandom()
{
    const float PI = 3.14159265f;
    const char* meshes[3] = { "the_die", "the_pawn", "the_sphere" };
    int ok = 0, blocked = 0, bad = 0, off = 0, picks = 0;
    AABB far_player; far_player.min = glm::vec3(100); far_player.max = glm::vec3(101);
    for (int trial = 0; trial < 300; ++trial)
    {
        BuildScene(g_Instances);
        int idx = -1;
        for (size_t i = 0; i < g_Instances.size(); ++i)
            if (g_Instances[i].grabbable && g_Instances[i].mesh == meshes[trial % 3]) { idx = (int)i; break; }
        Instance& o = g_Instances[idx];
        glm::vec3 la, lb; GetMeshLocalBBox(o.mesh, &la, &lb);
        o.tilt = RotationAboutAxis(RndAxis(), Rnd(0, PI));
        glm::vec3 eye(Rnd(-5, 5), Rnd(0.8f, 3.0f), Rnd(-3.8f, 3.8f));
        glm::vec3 c = eye + glm::vec3(Rnd(-1, 1), Rnd(-0.5f, 0.3f), Rnd(-1, 1));
        SetInstanceBodyCenter(o, la, lb, c);
        glm::vec3 to = c - eye;
        Camera cam; CameraInit(cam, eye, std::atan2(to.z, to.x), std::atan2(to.y, std::sqrt(to.x*to.x + to.z*to.z)));
        std::vector<int> owner; std::vector<AABB> boxes = SolidBoxes(&owner);
        GrabState g; GrabInit(g);
        if (!GrabTryPick(g, cam, 1.5f, 0, 0, boxes, owner) || g.instance != idx) continue;
        ++picks;
        glm::mat4 tilt0 = g_Instances[idx].tilt;
        for (int f = 0; f < 20; ++f)
        {
            cam.yaw += Rnd(-0.3f, 0.3f); cam.pitch = std::max(-1.2f, std::min(1.2f, cam.pitch + Rnd(-0.3f, 0.3f)));
            g.reach = Rnd(0.1f, 1.0f);
            if (rand() % 4 == 0) GrabRotate(g, Rnd(-1, 1));
            boxes = SolidBoxes(&owner);
            GrabUpdate(g, cam, boxes, owner, far_player, 1.0f/60);
            const Instance& h = g_Instances[idx];
            glm::vec3 fwd(CameraForward(cam).x, CameraForward(cam).y, CameraForward(cam).z);
            glm::vec3 d(g.dir_local.x*CameraRight(cam).x + g.dir_local.y*CameraUp(cam).x + g.dir_local.z*fwd.x,
                        g.dir_local.x*CameraRight(cam).y + g.dir_local.y*CameraUp(cam).y + g.dir_local.z*fwd.y,
                        g.dir_local.x*CameraRight(cam).z + g.dir_local.y*CameraUp(cam).z + g.dir_local.z*fwd.z);
            if (VecDiff(InstanceBodyCenter(h, la, lb), cam.eye + d*g.cur_distance) > 1e-3f) ++off;
            if (MatDiff(h.tilt, tilt0) != 0.0f) ++off;
            if (g.blocked) { ++blocked; continue; }
            AABB b = AABBExpand(InstanceCollisionBox(h, idx, la, lb), -GrabConfig::PLACE_MARGIN);
            if (AABBOverlapsAny(b, boxes, -1)) ++bad; else ++ok;
        }
    }
    printf("  %d pegas, %d quadros livres, %d bloqueados, %d penetrando, %d fora da mira\n", picks, ok, blocked, bad, off);
    CHECK(picks > 100 && bad == 0 && off == 0, "grab aleatorio: %d penetrando, %d fora da mira", bad, off);
}

int main()
{
    srand(12345);
    printf("== rigid: rotacoes\n");          TestRotation();
    printf("== rigid: aplicar/centro/reta\n"); TestApplyAndCenters();
    printf("== rigid: pose estavel\n");      TestStablePose();
    printf("== rigid: pose canonica\n");     TestCanonical();
    printf("== rigid: caixa de colisao\n");  TestAABB();
    printf("== cena\n");                     TestScene();
    printf("== grab com orientacao\n");      TestGrab();
    printf("== grab aleatorio tombado\n");   TestGrabRandom();
    printf("== t_a1: %d ok, %d falhas\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
