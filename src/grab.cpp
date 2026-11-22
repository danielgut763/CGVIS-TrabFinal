#include "grab.h"
#include "matrices.h"
#include "rigid.h"

#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// Auxiliares de vetor.
//
// Escritas a mao de proposito: o enunciado veta o uso de funcoes prontas de
// biblioteca para o calculo de camera e transformacoes, entao este modulo nao
// chama nenhum helper da GLM. Operacoes de 4 componentes usam norm(),
// dotproduct() e crossproduct() de "matrices.h", que sao nossas.
// ---------------------------------------------------------------------------
static float Dot3(const glm::vec3& a, const glm::vec3& b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

static float Length3(const glm::vec3& v)
{
    return std::sqrt(Dot3(v, v));
}

static glm::vec3 Normalize3(const glm::vec3& v)
{
    float n = Length3(v);
    if (n < 1e-12f)
        return glm::vec3(0.0f, 0.0f, -1.0f);
    return glm::vec3(v.x/n, v.y/n, v.z/n);
}

static glm::vec3 XYZ(const glm::vec4& v)
{
    return glm::vec3(v.x, v.y, v.z);
}

void GrabInit(GrabState& g)
{
    g.instance     = -1;
    g.dir_local    = glm::vec3(0.0f, 0.0f, 1.0f);
    g.d0           = 1.0f;
    g.s0           = 1.0f;
    g.yaw0         = 0.0f;
    g.was_solid    = true;
    g.yaw_offset   = 0.0f;
    g.reach        = 1.0f;
    g.cur_distance = 0.0f;
    g.cur_factor   = 1.0f;
    g.blocked      = false;
    g.vel_avg      = glm::vec3(0.0f);
    g.last_center  = glm::vec3(0.0f);
    g.has_last     = false;
}

// ---------------------------------------------------------------------------
// Raio do cursor, construido a partir da BASE DA CAMERA.
//
// gluUnProject e glm::unProject sao proibidas, e inverter a matriz View seria
// trabalho desnecessario: nos mesmos construimos essa base em
// Matrix_Camera_View, entao basta reusa-la.
//
// Um ponto do plano de projecao a distancia 1 da camera, em coordenadas de
// camera, tem componentes:
//
//     x_cam = ndc_x * aspect * tan(fov/2)
//     y_cam = ndc_y * tan(fov/2)
//     z_cam = -1                       (a camera olha para -Z)
//
// Levando para mundo pela base ortonormal (right, up, forward), e lembrando
// que o -Z de camera E o vetor forward:
//
//     dir = x_cam * right + y_cam * up + forward
// ---------------------------------------------------------------------------
void BuildCursorRay(const Camera& cam, float screen_ratio,
                    float ndc_x, float ndc_y,
                    glm::vec3* out_origin, glm::vec3* out_dir)
{
    glm::vec3 right   = XYZ(CameraRight(cam));
    glm::vec3 up      = XYZ(CameraUp(cam));
    glm::vec3 forward = XYZ(CameraForward(cam));

    float half = std::tan(cam.fov * 0.5f);
    float a = ndc_x * screen_ratio * half;
    float b = ndc_y * half;

    glm::vec3 dir(
        a*right.x + b*up.x + forward.x,
        a*right.y + b*up.y + forward.y,
        a*right.z + b*up.z + forward.z);

    if (out_origin) *out_origin = XYZ(CameraEyePosition(cam));
    if (out_dir)    *out_dir    = Normalize3(dir);
}

// Indice, dentro da lista de caixas, da caixa que pertence a instancia dada.
// Devolve -1 se ela nao estiver na lista.
static int BoxIndexOfInstance(const std::vector<int>& box_instance, int instance)
{
    for (size_t i = 0; i < box_instance.size(); ++i)
        if (box_instance[i] == instance)
            return (int)i;
    return -1;
}

// Posicao da ORIGEM da instancia tal que o centro da sua bounding box caia em
// "target_center", com o yaw e a escala dados e a inclinacao (tilt) atual da
// instancia. Os dois pontos so coincidem se a bbox da malha for centrada na
// origem do modelo, o que nem sempre acontece. Com a Model T*Ry*tilt*S:
//     centro = pos + Ry(yaw)*tilt*(scale*c_local)
//     pos    = target_center - Ry(yaw)*tilt*(scale*c_local)
// (SetInstanceBodyCenter, rigid.cpp). Ate o M7 era so Ry(yaw); com o tilt, um
// peao deitado na mao continua com o centro exatamente na mira.
static glm::vec3 InstancePosForCenter(const Instance& inst, float yaw, float scale,
                                      const glm::vec3& local_min, const glm::vec3& local_max,
                                      const glm::vec3& target_center)
{
    Instance probe = inst;
    probe.yaw   = yaw;
    probe.scale = glm::vec3(scale, scale, scale);
    SetInstanceBodyCenter(probe, local_min, local_max, target_center);
    return probe.pos;
}

// Caixa de COLISAO em mundo que o objeto teria se o centro da sua bbox
// estivesse em "center", com a escala e a rotacao dadas. Passa por
// InstanceCollisionBox, a mesma funcao que monta as caixas da fisica: se o
// pouso usasse outra caixa (a dos 8 cantos, por exemplo), um peao deitado
// seria solto com folga embaixo e cairia alguns centimetros, ou uma bola
// pareceria nao caber num vao em que cabe.
static AABB ObjectAABBAt(const Instance& inst, int idx,
                         const glm::vec3& local_min, const glm::vec3& local_max,
                         float yaw, float scale, const glm::vec3& center)
{
    Instance probe = inst;
    probe.yaw   = yaw;
    probe.scale = glm::vec3(scale, scale, scale);
    probe.pos   = InstancePosForCenter(inst, yaw, scale, local_min, local_max, center);

    return InstanceCollisionBox(probe, idx, local_min, local_max);
}

// ---------------------------------------------------------------------------
// Velocidade do objeto na mao (M8), para ele sair da mao com ela.
//
// A velocidade instantanea do centro, v = (c - c_anterior)/dt, e ruidosa: a
// distancia de pouso muda aos saltos quando o raio passa pela quina de um
// objeto. Por isso guardamos uma media exponencial (filtro passa-baixa de
// primeira ordem), que nao depende da taxa de quadros:
//     v_med <- v_med + a*(v - v_med),    a = 1 - exp(-dt/tau)
// Com tau = 0,08 s, um movimento so pesa depois de uns 3 ou 4 quadros a
// 60 Hz, e o que vale e o que aconteceu nos ultimos ~0,1 s antes de soltar.
// ---------------------------------------------------------------------------
static void TrackHeldVelocity(GrabState& g, const glm::vec3& center, float dt)
{
    if (dt <= 0.0f)
    {
        // Sem dt nao da para medir: o objeto sera solto parado (como no M7).
        g.vel_avg  = glm::vec3(0.0f);
        g.has_last = false;
        return;
    }

    if (g.has_last)
    {
        glm::vec3 v = (center - g.last_center) * (1.0f / dt);
        float a = 1.0f - std::exp(-dt / GrabConfig::RELEASE_VEL_TAU);
        g.vel_avg = g.vel_avg + (v - g.vel_avg) * a;
    }

    g.last_center = center;
    g.has_last    = true;
}

bool GrabTryPick(GrabState& g, Camera& cam, float screen_ratio,
                 float ndc_x, float ndc_y,
                 const std::vector<AABB>& boxes,
                 const std::vector<int>& box_instance)
{
    if (GrabIsHolding(g))
        return false;

    glm::vec3 origin, dir;
    BuildCursorRay(cam, screen_ratio, ndc_x, ndc_y, &origin, &dir);

    // Picking: o primeiro objeto atingido pelo raio. Se houver uma parede na
    // frente, e a parede que e atingida e nada e pego.
    RayHit hit = RaycastBoxes(origin, dir, boxes, -1);
    if (!hit.hit)
        return false;

    if (hit.index < 0 || hit.index >= (int)box_instance.size())
        return false;

    int idx = box_instance[hit.index];
    if (idx < 0 || idx >= (int)g_Instances.size())
        return false;

    Instance& inst = g_Instances[idx];
    if (!inst.grabbable)
        return false;

    // Centro do corpo (centro da bbox local em mundo), o mesmo ponto que
    // GrabUpdate mantem na mira. Para todas as formas ele coincide com o
    // centro da caixa de colisao, que fica de reserva.
    glm::vec3 center = AABBCenter(boxes[hit.index]);
    {
        glm::vec3 lmin, lmax;
        if (GetMeshLocalBBox(inst.mesh, &lmin, &lmax))
            center = InstanceBodyCenter(inst, lmin, lmax);
    }
    glm::vec3 v      = center - origin;
    float     d0     = Length3(v);
    if (d0 < GrabConfig::MIN_DISTANCE)
        return false;

    glm::vec3 dir0 = glm::vec3(v.x/d0, v.y/d0, v.z/d0);

    // Guardamos a direcao camera->objeto NA BASE DA CAMERA. Reconstruida a
    // cada quadro na base atual, ela faz o objeto permanecer exatamente no
    // mesmo ponto da tela enquanto o jogador olha em volta.
    glm::vec3 right   = XYZ(CameraRight(cam));
    glm::vec3 up      = XYZ(CameraUp(cam));
    glm::vec3 forward = XYZ(CameraForward(cam));

    g.dir_local = glm::vec3(Dot3(dir0, right),
                            Dot3(dir0, up),
                            Dot3(dir0, forward));

    g.d0         = d0;
    g.s0         = inst.scale.x;   // objetos pegaveis usam escala uniforme
    g.yaw0       = inst.yaw;
    g.yaw_offset = 0.0f;
    g.reach      = 1.0f;
    g.was_solid  = inst.solid;
    g.instance   = idx;
    g.blocked    = false;

    // Comeca a medir a velocidade da mao do zero.
    g.vel_avg    = glm::vec3(0.0f);
    g.has_last   = false;

    // O objeto pode ter sido pego no meio de um tombo, rolando ou caindo:
    // na mao ele nao tem velocidade nem estado de movimento. A inclinacao
    // (tilt) fica como esta: pegar um peao deitado mantem ele deitado.
    inst.vel  = glm::vec3(0.0f);
    inst.body = BodyState();

    // Enquanto esta na mao, o objeto sai da colisao (senao bloquearia o
    // proprio jogador) e o FOV fica travado: a invariante de tamanho aparente
    // so vale com o campo de visao constante.
    inst.solid   = false;
    cam.lock_fov = true;

    return true;
}

void GrabUpdate(GrabState& g, const Camera& cam,
                const std::vector<AABB>& boxes,
                const std::vector<int>& box_instance,
                const AABB& player_box,
                float dt)
{
    if (!GrabIsHolding(g))
        return;

    Instance& inst = g_Instances[g.instance];

    glm::vec3 local_min, local_max;
    if (!GetMeshLocalBBox(inst.mesh, &local_min, &local_max))
        return;

    const int skip = BoxIndexOfInstance(box_instance, g.instance);

    // Direcao do raio neste quadro: a direcao congelada no momento de pegar,
    // reconstruida na base ATUAL da camera.
    glm::vec3 right   = XYZ(CameraRight(cam));
    glm::vec3 up      = XYZ(CameraUp(cam));
    glm::vec3 forward = XYZ(CameraForward(cam));

    glm::vec3 dir = Normalize3(glm::vec3(
        g.dir_local.x*right.x + g.dir_local.y*up.x + g.dir_local.z*forward.x,
        g.dir_local.x*right.y + g.dir_local.y*up.y + g.dir_local.z*forward.y,
        g.dir_local.x*right.z + g.dir_local.y*up.z + g.dir_local.z*forward.z));

    glm::vec3 origin = XYZ(CameraEyePosition(cam));

    // -----------------------------------------------------------------------
    // 1) Ate onde a cena deixa o objeto ir, ao longo do raio.
    // -----------------------------------------------------------------------
    RayHit hit  = RaycastBoxes(origin, dir, boxes, skip);
    float  t_hit = hit.hit ? hit.t : 1.0e6f;
    t_hit -= GrabConfig::PLACE_MARGIN;
    if (t_hit < 0.0f)
        t_hit = 0.0f;

    float yaw = g.yaw0 + g.yaw_offset;

    // -----------------------------------------------------------------------
    // 2) Onde o objeto pousa.
    //
    // O professor comentou exatamente este ponto: NAO se posiciona o objeto no
    // ponto atingido pelo raio. O objeto tem volume, e esse volume cresce
    // junto com a distancia, o que realimenta o problema.
    //
    // Seja r_eff a meia-extensao efetiva do objeto ao longo do raio quando ele
    // esta a d0 com a rotacao atual "yaw". Como a escala e proporcional a
    // distancia, a uma distancia d essa meia-extensao vale r_eff*(d/d0). Para
    // a face da frente apenas ENCOSTAR na superficie atingida (normal
    // hit.normal), e nao atravessa-la nem em angulos rasantes:
    //
    //     d + r_eff*(d/d0) <= t_hit
    //     d * (1 + r_eff/d0) <= t_hit
    //     d <= t_hit / (1 + r_eff/d0)
    //
    // Calculamos r_eff combinando a meia-extensao na direcao do raio com a
    // projecao sobre a normal da face atingida dividida por |dir . normal|, o
    // que mantem o contato exato em forma fechada em qualquer angulo de visada
    // (evitando saltos ou flickering da busca binaria ao deslizar pelo piso).
    // -----------------------------------------------------------------------
    AABB  box0  = ObjectAABBAt(inst, g.instance, local_min, local_max, yaw, g.s0, glm::vec3(0.0f));
    float r_eff = AABBHalfExtentAlongDir(box0, dir);
    if (hit.hit)
    {
        float cos_n = std::fabs(Dot3(dir, hit.normal));
        if (cos_n > 1e-4f)
        {
            float r_plane = AABBHalfExtentAlongDir(box0, hit.normal) / cos_n;
            if (r_plane > r_eff)
                r_eff = r_plane;
        }
    }

    float d_wall = t_hit / (1.0f + r_eff / g.d0);

    // Limites de escala, para o objeto nao virar poeira nem engolir a fase
    // (objeto gigante e distante tambem castiga a precisao do z-buffer).
    float d_cap = g.d0 * GrabConfig::MAX_FACTOR;
    float d_min = g.d0 * GrabConfig::MIN_FACTOR;
    if (d_min < GrabConfig::MIN_DISTANCE)
        d_min = GrabConfig::MIN_DISTANCE;

    float d_max        = std::min(d_wall, d_cap);
    bool  too_close    = (d_max < d_min);
    float d            = d_min;

    if (too_close)
    {
        // A parede atingida esta mais perto que d_min: nao empurramos d para
        // d_min (o que atravessaria uma parede fina de 0.2 m e jogaria o
        // objeto para fora da sala); mantemos o objeto aquem da parede e
        // sinalizamos que nao ha espaco para soltar.
        d = std::max(0.25f, d_max);
    }
    else
    {
        // O jogador ainda escolhe, com a roda do mouse, o quanto do alcance
        // quer usar. Continua sendo escala derivada da distancia, so que a
        // distancia tambem e controlavel.
        d = g.reach * d_max;
        if (d < d_min) d = d_min;
        if (d > d_max) d = d_max;
    }

    // -----------------------------------------------------------------------
    // 3) Refinamento lateral e verificacao contra o corpo do jogador.
    //
    // A forma fechada acima so garante a direcao do raio. Num corredor
    // estreito, perto de um canto, ou com outro objeto ao lado, o volume ainda
    // pode penetrar de lado. Aqui recuamos por busca binaria ate caber.
    // -----------------------------------------------------------------------
    AABB player_padded = AABBExpand(player_box, 0.02f);
    {
        glm::vec3 center = origin + dir*d;
        AABB      probe  = ObjectAABBAt(inst, g.instance, local_min, local_max,
                                        yaw, g.s0*(d/g.d0), center);

        if (!too_close && AABBOverlapsAny(probe, boxes, skip))
        {
            float lo = d_min;
            float hi = d;

            for (int k = 0; k < GrabConfig::REFINE_STEPS; ++k)
            {
                float mid = 0.5f*(lo + hi);
                glm::vec3 c = origin + dir*mid;
                AABB b = ObjectAABBAt(inst, g.instance, local_min, local_max,
                                      yaw, g.s0*(mid/g.d0), c);

                if (AABBOverlapsAny(b, boxes, skip))
                    hi = mid;
                else
                    lo = mid;
            }

            d = lo;
        }

        glm::vec3 c = origin + dir*d;
        AABB b = ObjectAABBAt(inst, g.instance, local_min, local_max,
                              yaw, g.s0*(d/g.d0), c);
        AABB b_shrunk = AABBExpand(b, -GrabConfig::PLACE_MARGIN);

        g.blocked = too_close
                 || AABBOverlapsAny(b_shrunk, boxes, skip)
                 || AABBOverlap(b, player_padded);
    }

    // -----------------------------------------------------------------------
    // 4) Aplica a transformacao na instancia.
    //
    // s(d) = s0 * (d/d0) e a escala que mantem o tamanho aparente constante
    // sob projecao perspectiva.
    // -----------------------------------------------------------------------
    float factor = d / g.d0;
    float scale  = g.s0 * factor;

    glm::vec3 center  = origin + dir*d;

    inst.pos   = InstancePosForCenter(inst, yaw, scale, local_min, local_max, center);
    inst.yaw   = yaw;
    inst.scale = glm::vec3(scale, scale, scale);

    g.cur_distance = d;
    g.cur_factor   = factor;

    TrackHeldVelocity(g, center, dt);
}

bool GrabTryRelease(GrabState& g, Camera& cam,
                    const std::vector<AABB>& boxes,
                    const std::vector<int>& box_instance,
                    const AABB& player_box)
{
    if (!GrabIsHolding(g))
        return false;

    if (g.blocked)
        return false;

    Instance& inst = g_Instances[g.instance];

    glm::vec3 local_min, local_max;
    if (GetMeshLocalBBox(inst.mesh, &local_min, &local_max))
    {
        const int skip = BoxIndexOfInstance(box_instance, g.instance);

        // A mesma caixa da fisica (InstanceCollisionBox): o que cabe aqui
        // cabe la.
        AABB box = InstanceCollisionBox(inst, g.instance, local_min, local_max);

        // Encolhemos um pouco a caixa antes de testar contra a cena para que
        // apenas ENCOSTAR numa parede nao conte como penetracao, e testamos
        // tambem contra a AABB do jogador para nunca prende-lo dentro do objeto.
        if (AABBOverlapsAny(AABBExpand(box, -GrabConfig::PLACE_MARGIN), boxes, skip) ||
            AABBOverlap(box, AABBExpand(player_box, 0.02f)))
        {
            // Nao cabe aqui: continua na mao, para nao deixar o objeto preso
            // dentro da geometria ou travando o jogador.
            g.blocked = true;
            return false;
        }
    }

    // Velocidade de saida: a media da mao, com modulo limitado (ver
    // GrabConfig::RELEASE_MAX_SPEED para o porque do limite). Em look-at o
    // objeto nao acompanha a camera (GrabUpdate nao roda), entao sai parado.
    glm::vec3 v_out(0.0f);
    if (GrabConfig::INHERIT_RELEASE_VELOCITY && cam.mode == CAM_FIRST_PERSON && g.has_last)
    {
        v_out = g.vel_avg;
        float speed = Length3(v_out);
        if (speed > GrabConfig::RELEASE_MAX_SPEED)
            v_out = v_out * (GrabConfig::RELEASE_MAX_SPEED / speed);
    }

    // A partir daqui a fisica assume (physics.cpp): o objeto comeca no ar
    // (MOTION_FALL) e, se ja estiver encostado no apoio, pousa no primeiro
    // passo e assenta ou sai rolando.
    inst.solid       = g.was_solid;
    inst.vel         = v_out;
    inst.body        = BodyState();
    inst.body.motion = MOTION_FALL;
    cam.lock_fov     = false;

    g.vel_avg  = glm::vec3(0.0f);
    g.has_last = false;

    g.instance = -1;
    g.blocked  = false;
    return true;
}

void GrabRotate(GrabState& g, float delta_yaw)
{
    if (!GrabIsHolding(g))
        return;

    g.yaw_offset += delta_yaw;

    // Atualiza tambem a instancia imediatamente ao redor do centro da sua
    // AABB, permitindo girar o objeto na mao mesmo enquanto inspecionado pela
    // camera look-at.
    if (g.instance >= 0 && g.instance < (int)g_Instances.size())
    {
        Instance& inst = g_Instances[g.instance];
        glm::vec3 local_min, local_max;
        if (GetMeshLocalBBox(inst.mesh, &local_min, &local_max))
        {
            // Gira em torno do Y do mundo passando pelo centro do corpo:
            // como a Model e T*Ry(yaw)*tilt*S, mudar o yaw gira o objeto
            // inteiro (inclinado ou nao) em torno da vertical.
            glm::vec3 center = InstanceBodyCenter(inst, local_min, local_max);
            float     yaw    = g.yaw0 + g.yaw_offset;

            inst.pos = InstancePosForCenter(inst, yaw, inst.scale.x, local_min, local_max, center);
            inst.yaw = yaw;
        }
        else
        {
            inst.yaw = g.yaw0 + g.yaw_offset;
        }
    }
}

void GrabAdjustReach(GrabState& g, float wheel)
{
    if (!GrabIsHolding(g))
        return;

    g.reach += wheel * GrabConfig::REACH_STEP;

    if (g.reach < GrabConfig::MIN_REACH) g.reach = GrabConfig::MIN_REACH;
    if (g.reach > 1.0f)                  g.reach = 1.0f;
}
