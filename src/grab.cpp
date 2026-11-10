#include "grab.h"
#include "matrices.h"

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
    g.r0           = 0.0f;
    g.yaw0         = 0.0f;
    g.was_solid    = true;
    g.yaw_offset   = 0.0f;
    g.reach        = 1.0f;
    g.cur_distance = 0.0f;
    g.cur_factor   = 1.0f;
    g.blocked      = false;
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
// "target_center". Os dois pontos so coincidem se a bbox da malha for centrada
// na origem do modelo, o que nem sempre acontece.
static glm::vec3 InstancePosForCenter(float yaw, float scale,
                                      const glm::vec3& c_local,
                                      const glm::vec3& target_center)
{
    // Rotacao em torno de Y, na mesma convencao de Matrix_Rotate_Y.
    float c = std::cos(yaw);
    float s = std::sin(yaw);

    glm::vec3 v = c_local * scale;
    glm::vec3 off(c*v.x + s*v.z,
                  v.y,
                  -s*v.x + c*v.z);

    return target_center - off;
}

// AABB em mundo que o objeto teria se o centro da sua bbox estivesse em
// "center", com a escala e a rotacao dadas.
static AABB ObjectAABBAt(const Instance& inst,
                         const glm::vec3& local_min, const glm::vec3& local_max,
                         float yaw, float scale, const glm::vec3& center)
{
    glm::vec3 c_local = (local_min + local_max) * 0.5f;

    Instance probe = inst;
    probe.yaw   = yaw;
    probe.scale = glm::vec3(scale, scale, scale);
    probe.pos   = InstancePosForCenter(yaw, scale, c_local, center);

    return AABBTransform(local_min, local_max, InstanceModelMatrix(probe));
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

    glm::vec3 center = AABBCenter(boxes[hit.index]);
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
    g.r0         = AABBHalfExtentAlongDir(boxes[hit.index], dir0);
    g.yaw0       = inst.yaw;
    g.yaw_offset = 0.0f;
    g.reach      = 1.0f;
    g.was_solid  = inst.solid;
    g.instance   = idx;
    g.blocked    = false;

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
                const AABB& player_box)
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

    // -----------------------------------------------------------------------
    // 2) Onde o objeto pousa.
    //
    // O professor comentou exatamente este ponto: NAO se posiciona o objeto no
    // ponto atingido pelo raio. O objeto tem volume, e esse volume cresce
    // junto com a distancia, o que realimenta o problema.
    //
    // Seja r0 a meia-extensao do objeto ao longo do raio quando ele estava a
    // d0. Como a escala e proporcional a distancia, a uma distancia d essa
    // meia-extensao vale r0*(d/d0). Para a face da frente apenas ENCOSTAR na
    // superficie, e nao atravessa-la:
    //
    //     d + r0*(d/d0) <= t_hit
    //     d * (1 + r0/d0) <= t_hit
    //     d <= t_hit / (1 + r0/d0)
    //
    // Isto e um sphere cast de uma esfera que cresce proporcionalmente a
    // distancia, e resolve o caso da parede de frente em forma fechada, sem
    // nenhuma iteracao.
    // -----------------------------------------------------------------------
    float d_wall = t_hit / (1.0f + g.r0 / g.d0);

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

    float yaw = g.yaw0 + g.yaw_offset;

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
        AABB      probe  = ObjectAABBAt(inst, local_min, local_max,
                                        yaw, g.s0*(d/g.d0), center);

        if (!too_close && AABBOverlapsAny(probe, boxes, skip))
        {
            float lo = d_min;
            float hi = d;

            for (int k = 0; k < GrabConfig::REFINE_STEPS; ++k)
            {
                float mid = 0.5f*(lo + hi);
                glm::vec3 c = origin + dir*mid;
                AABB b = ObjectAABBAt(inst, local_min, local_max,
                                      yaw, g.s0*(mid/g.d0), c);

                if (AABBOverlapsAny(b, boxes, skip))
                    hi = mid;
                else
                    lo = mid;
            }

            d = lo;
        }

        glm::vec3 c = origin + dir*d;
        AABB b = ObjectAABBAt(inst, local_min, local_max,
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
    glm::vec3 c_local = (local_min + local_max) * 0.5f;

    inst.yaw   = yaw;
    inst.scale = glm::vec3(scale, scale, scale);
    inst.pos   = InstancePosForCenter(yaw, scale, c_local, center);

    g.cur_distance = d;
    g.cur_factor   = factor;
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

        AABB box = AABBTransform(local_min, local_max, InstanceModelMatrix(inst));

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

    inst.solid   = g.was_solid;
    inst.vel_y   = 0.0f;   // a partir daqui a gravidade assume (physics.cpp)
    cam.lock_fov = false;

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
            AABB cur_box      = AABBTransform(local_min, local_max, InstanceModelMatrix(inst));
            glm::vec3 center  = AABBCenter(cur_box);
            glm::vec3 c_local = (local_min + local_max) * 0.5f;
            float yaw         = g.yaw0 + g.yaw_offset;

            inst.yaw = yaw;
            inst.pos = InstancePosForCenter(yaw, inst.scale.x, c_local, center);
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
