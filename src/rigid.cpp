#include "rigid.h"

#include <algorithm>
#include <cmath>

#include "matrices.h"
#include "grab.h"   // GetMeshLocalBBox (implementada em main.cpp)

// ---------------------------------------------------------------------------
// Orientacao livre dos objetos pegaveis (M8). Ver rigid.h para as convencoes.
//
// Lembrete de notacao: glm::mat4 e "column-major", entao M[c] e a COLUNA c e
// M[c][r] e o elemento da linha r, coluna c. Numa matriz de rotacao R, a
// coluna c e a imagem do eixo local e_c:  R*e_c = R[c]. E por isso que a
// direcao em mundo de um eixo local do objeto e simplesmente uma coluna de
// Ry(yaw)*tilt (a escala S vem antes e so muda o comprimento).
// ---------------------------------------------------------------------------

namespace
{
    const float PI = 3.14159265358979f;

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

    float Length3(const glm::vec3& v)
    {
        return std::sqrt(Dot3(v, v));
    }

    // Normaliza; se o vetor for praticamente nulo devolve "fallback".
    glm::vec3 Normalize3(const glm::vec3& v, const glm::vec3& fallback)
    {
        float n = Length3(v);
        if (n < 1e-12f)
            return fallback;
        return v * (1.0f / n);
    }

    glm::vec3 Col3(const glm::mat4& M, int c)
    {
        return glm::vec3(M[c][0], M[c][1], M[c][2]);
    }

    void SetCol3(glm::mat4& M, int c, const glm::vec3& v)
    {
        M[c][0] = v.x;
        M[c][1] = v.y;
        M[c][2] = v.z;
        M[c][3] = 0.0f;
    }

    // Matriz * vetor so com a parte 3x3 (direcao, w = 0).
    glm::vec3 MulDir(const glm::mat4& M, const glm::vec3& v)
    {
        glm::vec4 r = M * glm::vec4(v.x, v.y, v.z, 0.0f);
        return glm::vec3(r.x, r.y, r.z);
    }

    // Leva um angulo para (-pi, pi].
    float WrapAngle(float a)
    {
        while (a >   PI) a -= 2.0f*PI;
        while (a <= -PI) a += 2.0f*PI;
        return a;
    }

    // Menor rotacao que leva a direcao unitaria "from" ate a direcao unitaria
    // "to":
    //     eixo   = (from x to) / |from x to|
    //     angulo = atan2(|from x to|, from . to)
    // (atan2 em vez de acos porque acos perde precisao perto de 0 e de pi,
    // justamente onde o objeto esta quase assentado.)
    // Para from e to opostos o eixo e qualquer perpendicular a "from".
    void RotationBetween(const glm::vec3& from, const glm::vec3& to,
                         glm::vec3* out_axis, float* out_angle)
    {
        glm::vec3 c = Cross3(from, to);
        float     s = Length3(c);
        float     d = Dot3(from, to);

        if (s < 1e-7f)
        {
            if (d > 0.0f)
            {
                *out_axis  = glm::vec3(0.0f, 1.0f, 0.0f);
                *out_angle = 0.0f;
            }
            else
            {
                // Antiparalelos: gira meia volta em torno de qualquer eixo
                // perpendicular. Escolhe o eixo de mundo menos alinhado com
                // "from" para o produto vetorial ser bem condicionado.
                glm::vec3 helper = (std::fabs(from.x) < 0.9f) ? glm::vec3(1.0f, 0.0f, 0.0f)
                                                              : glm::vec3(0.0f, 0.0f, 1.0f);
                *out_axis  = Normalize3(Cross3(from, helper), glm::vec3(0.0f, 0.0f, 1.0f));
                *out_angle = PI;
            }
            return;
        }

        *out_axis  = c * (1.0f / s);
        *out_angle = std::atan2(s, d);
    }

    // Recalcula inst.pos para manter o centro da bbox em "center". Usada
    // pelas funcoes que nao recebem a bbox (CanonicalizePose).
    void KeepCenterWhile(Instance& inst, const glm::mat4& old_model)
    {
        glm::vec3 lmin, lmax;
        if (!GetMeshLocalBBox(inst.mesh, &lmin, &lmax))
            return;

        glm::vec3 c_local = (lmin + lmax) * 0.5f;
        glm::vec4 c = old_model * glm::vec4(c_local.x, c_local.y, c_local.z, 1.0f);
        SetInstanceBodyCenter(inst, lmin, lmax, glm::vec3(c.x, c.y, c.z));
    }

    // -----------------------------------------------------------------------
    // Limite de equilibrio do peao em pe, tirado da geometria (bbox local).
    //
    // Um corpo apoiado numa base plana so volta sozinho para a base se a
    // vertical que passa pelo centro de massa cair DENTRO da base. Inclinado
    // de um angulo t, ele se apoia na borda da base. No referencial do
    // corpo, o vetor da borda ate o centro de massa e (r_base, h_cm): r_base
    // para dentro, ao longo da base, e h_cm para cima, ao longo do eixo.
    // Esse vetor fica na vertical (centro exatamente acima da borda, ponto
    // de tombar) quando a inclinacao vale
    //
    //     tan(t_lim) = r_base / h_cm,    t_lim = atan(r_base / h_cm)
    //
    // Abaixo de t_lim o peso puxa o peao de volta para a base; acima, ele
    // tomba e a pose estavel mais proxima e deitado.
    //
    // Com a bbox do peao (corpo de revolucao em torno do Y local, base em
    // lmin.y):
    //     r_base = metade do maior lado XZ da bbox, na escala do objeto
    //     h_cm   = (centro da bbox).y - lmin.y, na escala do objeto
    // (o centro da bbox e o centro de massa que a fisica usa). Para a malha
    // atual, r_base/h_cm = 0.335/0.5 e t_lim ~ 34 graus. Devolve cos(t_lim),
    // para comparar direto com u.y = cos(inclinacao). Sem bbox, usa 45 graus.
    // -----------------------------------------------------------------------
    float PawnStandCos(const Instance& inst)
    {
        glm::vec3 lmin, lmax;
        if (!GetMeshLocalBBox(inst.mesh, &lmin, &lmax))
            return 0.70710678f;

        float r_base = 0.5f * std::max((lmax.x - lmin.x) * std::fabs(inst.scale.x),
                                       (lmax.z - lmin.z) * std::fabs(inst.scale.z));
        float h_cm   = 0.5f * (lmax.y - lmin.y) * std::fabs(inst.scale.y);
        if (h_cm < 1e-9f)
            return 0.70710678f;

        // cos(atan(r/h)) = h / sqrt(h^2 + r^2)
        return h_cm / std::sqrt(h_cm*h_cm + r_base*r_base);
    }
}

// ---------------------------------------------------------------------------
// Formula de Rodrigues. Para o eixo unitario a e o angulo t, com c = cos t,
// s = sin t e K a matriz do produto vetorial (K*v = a x v):
//
//     R = c*I + (1 - c)*a*a^T + s*K,      K = [  0  -az   ay ]
//                                             [  az   0  -ax ]
//                                             [ -ay  ax    0 ]
//
// Elemento a elemento (linha r, coluna k):
//     R_rk = c*delta_rk + (1 - c)*a_r*a_k + s*K_rk
//
// Com a = (0,0,1) sai exatamente Matrix_Rotate_Z: e a regra da mao direita
// (angulo positivo leva +X para +Y).
// ---------------------------------------------------------------------------
glm::mat4 RotationAboutAxis(const glm::vec3& axis, float angle)
{
    float n = Length3(axis);
    if (n < 1e-12f)
        return Matrix_Identity();

    glm::vec3 a = axis * (1.0f / n);
    float c = std::cos(angle);
    float s = std::sin(angle);
    float t = 1.0f - c;

    return Matrix(
        t*a.x*a.x + c,      t*a.x*a.y - s*a.z,  t*a.x*a.z + s*a.y,  0.0f,
        t*a.x*a.y + s*a.z,  t*a.y*a.y + c,      t*a.y*a.z - s*a.x,  0.0f,
        t*a.x*a.z - s*a.y,  t*a.y*a.z + s*a.x,  t*a.z*a.z + c,      0.0f,
        0.0f,               0.0f,               0.0f,               1.0f
    );
}

// ---------------------------------------------------------------------------
// Gram-Schmidt nas colunas x, y, z:
//     x' = x / |x|
//     y' = (y - (y.x')x') / |...|
//     z' = x' x y'            (produto vetorial: garante base DIREITA, det = +1)
// Uma rotacao composta milhares de vezes (bola rolando a 240 Hz) acumula erro
// de arredondamento e deixa de ser ortonormal: o objeto comecaria a se
// deformar (cisalhar/escalar). Reortonormalizar a cada passo custa poucas
// operacoes e mata esse acumulo.
// ---------------------------------------------------------------------------
void OrthonormalizeRotation(glm::mat4& R)
{
    glm::vec3 x = Normalize3(Col3(R, 0), glm::vec3(1.0f, 0.0f, 0.0f));
    glm::vec3 y = Col3(R, 1);
    y = y - x * Dot3(y, x);

    if (Length3(y) < 1e-6f)
    {
        // Coluna y degenerada (paralela a x): reconstroi a partir de z.
        glm::vec3 z0 = Col3(R, 2);
        y = Cross3(z0, x);
        if (Length3(y) < 1e-6f)
        {
            glm::vec3 helper = (std::fabs(x.y) < 0.9f) ? glm::vec3(0.0f, 1.0f, 0.0f)
                                                       : glm::vec3(0.0f, 0.0f, 1.0f);
            y = helper - x * Dot3(helper, x);
        }
    }
    y = Normalize3(y, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::vec3 z = Cross3(x, y);

    SetCol3(R, 0, x);
    SetCol3(R, 1, y);
    SetCol3(R, 2, z);
}

glm::mat4 InstanceOrientation(const Instance& inst)
{
    return Matrix_Rotate_Y(inst.yaw) * inst.tilt;
}

// ---------------------------------------------------------------------------
// Queremos a nova orientacao O' = Rot * O, com O = Ry(yaw)*tilt, mas
// guardando o yaw como esta (Q/E na mao continuam girando em torno do Y do
// mundo). Isolando o novo tilt:
//
//     Ry(yaw) * tilt' = Rot * Ry(yaw) * tilt
//               tilt' = Ry(-yaw) * Rot * Ry(yaw) * tilt
//
// Ry(-yaw)*Rot*Ry(yaw) e a mesma rotacao Rot escrita no referencial do objeto
// ja girado pelo yaw (eixo Ry(-yaw)*axis, mesmo angulo).
// ---------------------------------------------------------------------------
void ApplyWorldRotation(Instance& inst, const glm::vec3& axis, float angle)
{
    if (angle == 0.0f)
        return;

    glm::mat4 R = RotationAboutAxis(axis, angle);
    inst.tilt = Matrix_Rotate_Y(-inst.yaw) * R * Matrix_Rotate_Y(inst.yaw) * inst.tilt;
    OrthonormalizeRotation(inst.tilt);
}

// ---------------------------------------------------------------------------
// Centro da bbox local levado para mundo pela Model completa:
//     c_mundo = T * Ry(yaw) * tilt * S * c_local = pos + O*S*c_local
// e, ao contrario, para o centro cair em "center":
//     pos = center - O*S*c_local
// ---------------------------------------------------------------------------
glm::vec3 InstanceBodyCenter(const Instance& inst, const glm::vec3& lmin, const glm::vec3& lmax)
{
    glm::vec3 c_local = (lmin + lmax) * 0.5f;
    glm::vec4 c = InstanceModelMatrix(inst) * glm::vec4(c_local.x, c_local.y, c_local.z, 1.0f);
    return glm::vec3(c.x, c.y, c.z);
}

void SetInstanceBodyCenter(Instance& inst, const glm::vec3& lmin, const glm::vec3& lmax,
                           const glm::vec3& center)
{
    glm::vec3 c_local = (lmin + lmax) * 0.5f;
    glm::vec3 scaled(c_local.x * inst.scale.x,
                     c_local.y * inst.scale.y,
                     c_local.z * inst.scale.z);
    inst.pos = center - MulDir(InstanceOrientation(inst), scaled);
}

void RotateAboutCenter(Instance& inst, const glm::vec3& lmin, const glm::vec3& lmax,
                       const glm::vec3& axis, float angle)
{
    glm::vec3 c = InstanceBodyCenter(inst, lmin, lmax);
    ApplyWorldRotation(inst, axis, angle);
    SetInstanceBodyCenter(inst, lmin, lmax, c);
}

// ---------------------------------------------------------------------------
// Rotacao rigida em torno da reta (pivot, axis). Todo ponto p do corpo vai
// para  p' = pivot + R*(p - pivot). Aplicado ao centro:
//     c' = pivot + R*(c - pivot)
// e a orientacao gira pela mesma R (O' = R*O). Como pos e so "c menos um
// vetor que gira com O", recolocar o centro em c' com a orientacao nova da
// exatamente o mesmo resultado que girar todos os pontos.
// ---------------------------------------------------------------------------
void RotateAboutLine(Instance& inst, const glm::vec3& lmin, const glm::vec3& lmax,
                     const glm::vec3& pivot, const glm::vec3& axis, float angle)
{
    if (angle == 0.0f)
        return;

    glm::mat4 R = RotationAboutAxis(axis, angle);
    glm::vec3 c = InstanceBodyCenter(inst, lmin, lmax);
    glm::vec3 c_new = pivot + MulDir(R, c - pivot);

    ApplyWorldRotation(inst, axis, angle);
    SetInstanceBodyCenter(inst, lmin, lmax, c_new);
}

glm::vec3 InstanceLocalAxisWorld(const Instance& inst, int local_axis)
{
    if (local_axis < 0) local_axis = 0;
    if (local_axis > 2) local_axis = 2;
    glm::mat4 O = InstanceOrientation(inst);
    return Normalize3(Col3(O, local_axis), glm::vec3(0.0f, 1.0f, 0.0f));
}

bool PawnIsLying(const Instance& inst)
{
    return std::fabs(InstanceLocalAxisWorld(inst, 1).y) < 0.5f;
}

// ---------------------------------------------------------------------------
// Pose estavel mais proxima.
//
// BOX: entre os 6 eixos locais (+-X, +-Y, +-Z) em mundo, o mais alinhado com
//   +Y e a normal da face que vai ficar para cima (a oposta fica no chao).
//   Basta leva-lo exatamente a +Y pela menor rotacao. O pior caso e o cubo
//   apoiado num vertice: o melhor eixo faz acos(1/sqrt(3)) ~ 54,7 graus com a
//   vertical, entao w nunca e antiparalelo a +Y.
//
// PAWN: u = eixo local +Y em mundo (de revolucao, base em -Y).
//   - inclinacao menor que o limite de equilibrio t_lim = atan(r_base/h_cm)
//     (centro de massa ainda acima da base, ver PawnStandCos; ~34 graus):
//     levanta, u -> +Y;
//   - senao deita: leva u para a sua projecao horizontal h = (ux, 0, uz)/|..|,
//     o que gira asin(|u.y|), a menor rotacao que zera u.y. De cabeca para
//     baixo tambem deita (o topo arredondado nao e uma base); se u esta
//     exatamente na vertical para baixo, h vem do eixo local X.
//
// SPHERE: qualquer pose e estavel.
// ---------------------------------------------------------------------------
bool NearestStablePose(const Instance& inst, float tolerance,
                       glm::vec3* out_axis, float* out_angle)
{
    glm::vec3 axis(0.0f, 1.0f, 0.0f);
    float     angle = 0.0f;
    const glm::vec3 up(0.0f, 1.0f, 0.0f);

    glm::mat4 O = InstanceOrientation(inst);

    if (inst.shape == SHAPE_BOX)
    {
        glm::vec3 best = Col3(O, 1);
        float     best_y = -2.0f;
        for (int k = 0; k < 3; ++k)
        {
            glm::vec3 w = Normalize3(Col3(O, k), glm::vec3(0.0f, 1.0f, 0.0f));
            if ( w.y > best_y) { best_y =  w.y; best =  w; }
            if (-w.y > best_y) { best_y = -w.y; best = -w; }
        }
        RotationBetween(best, up, &axis, &angle);
    }
    else if (inst.shape == SHAPE_PAWN)
    {
        glm::vec3 u = Normalize3(Col3(O, 1), up);

        if (u.y >= PawnStandCos(inst))
        {
            RotationBetween(u, up, &axis, &angle);
        }
        else
        {
            glm::vec3 h(u.x, 0.0f, u.z);
            if (Length3(h) < 1e-4f)
            {
                glm::vec3 x = Col3(O, 0);
                h = glm::vec3(x.x, 0.0f, x.z);
            }
            h = Normalize3(h, glm::vec3(1.0f, 0.0f, 0.0f));
            RotationBetween(u, h, &axis, &angle);
        }
    }
    // SHAPE_SPHERE: angle continua 0.

    if (angle <= tolerance)
    {
        if (out_axis)  *out_axis  = glm::vec3(0.0f, 1.0f, 0.0f);
        if (out_angle) *out_angle = 0.0f;
        return false;
    }

    if (out_axis)  *out_axis  = axis;
    if (out_angle) *out_angle = angle;
    return true;
}

// ---------------------------------------------------------------------------
// Forma canonica de uma pose assentada.
//
// BOX: a orientacao assentada e O = Ry(phi) * C, com C uma das 24 rotacoes do
// cubo (matriz de permutacao com sinais: cada coluna e +-X, +-Y ou +-Z) e
// phi o giro livre em torno da vertical. Reconstruimos C com entradas
// EXATAS, para que milhares de tombos nao acumulem erro em tilt:
//   1. k, sg: o eixo local cuja direcao em mundo sg*O*e_k mais se alinha com
//      +Y. Entao C*e_k = sg*Y.
//   2. j: dos outros dois, o mais horizontal; v = projecao horizontal
//      normalizada de O*e_j. C*e_j e um eixo horizontal t (+-X ou +-Z) e
//      Ry(phi)*t = v. Escrevendo t = Ry(theta_t)*X e v = Ry(b)*X, com
//      b = atan2(-v.z, v.x) (pois Ry(b)*X = (cos b, 0, -sin b)):
//          phi = b - theta_t.
//      Das 4 escolhas de t, fica a que deixa phi mais perto do yaw atual
//      (assim um objeto que nunca tombou volta com tilt = I e o mesmo yaw).
//   3. m: o terceiro eixo, pela regra da mao direita. Se (k, j, m) e uma
//      permutacao ciclica de (0,1,2), e_m = e_k x e_j e C*e_m = C*e_k x C*e_j;
//      senao e_m = e_j x e_k.
// Produto vetorial de vetores com entradas 0/+-1 da entradas 0/+-1 exatas.
//
// PAWN em pe (dentro do limite de equilibrio, PawnStandCos): O = Ry(phi),
// entao tilt = I e o yaw recebe phi, lido da
// projecao horizontal do eixo local X.
//
// PAWN deitado e SPHERE: qualquer pose e valida, so reortonormaliza.
//
// Em todos os casos o centro da bbox fica onde estava.
// ---------------------------------------------------------------------------
void CanonicalizePose(Instance& inst)
{
    const glm::mat4 old_model = InstanceModelMatrix(inst);
    const glm::mat4 O = InstanceOrientation(inst);

    if (inst.shape == SHAPE_BOX)
    {
        glm::vec3 col[3];
        for (int c = 0; c < 3; ++c)
            col[c] = Normalize3(Col3(O, c), glm::vec3(0.0f));

        // 1) eixo vertical
        int   k  = 1;
        float sg = 1.0f;
        float best = -2.0f;
        for (int c = 0; c < 3; ++c)
        {
            if ( col[c].y > best) { best =  col[c].y; k = c; sg =  1.0f; }
            if (-col[c].y > best) { best = -col[c].y; k = c; sg = -1.0f; }
        }

        // 2) eixo horizontal de referencia
        int o1 = (k + 1) % 3;
        int o2 = (k + 2) % 3;
        float h1 = col[o1].x*col[o1].x + col[o1].z*col[o1].z;
        float h2 = col[o2].x*col[o2].x + col[o2].z*col[o2].z;
        int   j  = (h1 >= h2) ? o1 : o2;
        int   m  = 3 - k - j;

        glm::vec3 v = Normalize3(glm::vec3(col[j].x, 0.0f, col[j].z),
                                 glm::vec3(1.0f, 0.0f, 0.0f));
        float b = std::atan2(-v.z, v.x);

        // Eixos horizontais candidatos t e seus angulos theta_t (t = Ry(theta)*X).
        const glm::vec3 t_dir[4] = {
            glm::vec3( 1.0f, 0.0f,  0.0f),   // theta = 0
            glm::vec3( 0.0f, 0.0f, -1.0f),   // theta = pi/2
            glm::vec3(-1.0f, 0.0f,  0.0f),   // theta = pi
            glm::vec3( 0.0f, 0.0f,  1.0f)    // theta = -pi/2
        };
        const float t_theta[4] = { 0.0f, 0.5f*PI, PI, -0.5f*PI };

        int   best_t   = 0;
        float best_err = 1e9f;
        for (int q = 0; q < 4; ++q)
        {
            float err = std::fabs(WrapAngle(b - t_theta[q] - inst.yaw));
            if (err < best_err) { best_err = err; best_t = q; }
        }
        float phi = inst.yaw + WrapAngle(b - t_theta[best_t] - inst.yaw);

        // 3) monta C com entradas exatas
        glm::vec3 Ck(0.0f, sg, 0.0f);
        glm::vec3 Cj = t_dir[best_t];
        bool cyclic = ((k + 1) % 3 == j);
        glm::vec3 Cm = cyclic ? Cross3(Ck, Cj) : Cross3(Cj, Ck);

        glm::mat4 C(1.0f);
        // "+ 0.0f" troca -0 por +0, so por higiene na hora de imprimir.
        SetCol3(C, k, Ck + glm::vec3(0.0f));
        SetCol3(C, j, Cj + glm::vec3(0.0f));
        SetCol3(C, m, Cm + glm::vec3(0.0f));

        inst.tilt = C;
        inst.yaw  = phi;
    }
    else if (inst.shape == SHAPE_PAWN && InstanceLocalAxisWorld(inst, 1).y >= PawnStandCos(inst))
    {
        glm::vec3 x = Col3(O, 0);
        glm::vec3 xh = Normalize3(glm::vec3(x.x, 0.0f, x.z), glm::vec3(1.0f, 0.0f, 0.0f));
        float b = std::atan2(-xh.z, xh.x);

        inst.yaw  = inst.yaw + WrapAngle(b - inst.yaw);
        inst.tilt = glm::mat4(1.0f);
    }
    else
    {
        OrthonormalizeRotation(inst.tilt);
    }

    KeepCenterWhile(inst, old_model);
}

// ---------------------------------------------------------------------------
// Caixa de colisao de um corpo inclinado (ver rigid.h).
//
// Cilindro de eixo unitario u, meia-altura h e raio r. Um ponto do cilindro
// e  c + a*u + rho*e,  com |a| <= h, e perpendicular a u e |rho| <= r. A
// maior coordenada i atingida e
//     max(a*u_i) + max(rho*e_i) = h*|u_i| + r*sqrt(1 - u_i^2),
// porque o maior e_i possivel entre os vetores unitarios perpendiculares a u
// e o comprimento da componente de e_i (eixo de mundo) perpendicular a u,
// sqrt(1 - u_i^2). Com u horizontal ao longo de X, por exemplo, sai
// (h, r, r): a caixa nao muda quando o peao gira em torno do proprio eixo.
// ---------------------------------------------------------------------------
AABB BodyWorldAABB(const Instance& inst, const glm::vec3& lmin, const glm::vec3& lmax)
{
    if (inst.shape == SHAPE_SPHERE)
    {
        glm::vec3 c = InstanceBodyCenter(inst, lmin, lmax);
        float sx = (lmax.x - lmin.x) * std::fabs(inst.scale.x);
        float sy = (lmax.y - lmin.y) * std::fabs(inst.scale.y);
        float sz = (lmax.z - lmin.z) * std::fabs(inst.scale.z);
        float r = 0.5f * std::max(sx, std::max(sy, sz));

        AABB box;
        box.min = c - glm::vec3(r);
        box.max = c + glm::vec3(r);
        return box;
    }

    if (inst.shape == SHAPE_PAWN)
    {
        glm::vec3 c = InstanceBodyCenter(inst, lmin, lmax);
        glm::vec3 u = InstanceLocalAxisWorld(inst, 1);

        float h  = 0.5f * (lmax.y - lmin.y) * std::fabs(inst.scale.y);
        float rx = 0.5f * (lmax.x - lmin.x) * std::fabs(inst.scale.x);
        float rz = 0.5f * (lmax.z - lmin.z) * std::fabs(inst.scale.z);
        float r  = std::max(rx, rz);

        glm::vec3 half;
        for (int i = 0; i < 3; ++i)
        {
            float ui = u[i];
            float perp = 1.0f - ui*ui;
            if (perp < 0.0f) perp = 0.0f;
            half[i] = std::fabs(ui)*h + r*std::sqrt(perp);
        }

        AABB box;
        box.min = c - half;
        box.max = c + half;
        return box;
    }

    return AABBTransform(lmin, lmax, InstanceModelMatrix(inst));
}
