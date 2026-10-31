#include "camera.h"
#include "matrices.h"
#include "scene.h"

#include <algorithm>
#include <cmath>

static const float PI_VAL    = 3.14159265f;
static const float MAX_PITCH = 1.50f; // ~86 graus: evita view_vector colinear com up (0,1,0)

void CameraInit(Camera& cam, const glm::vec3& eye_pos,
                float yaw_rad, float pitch_rad)
{
    cam.mode               = CAM_FIRST_PERSON;
    cam.eye                = eye_pos;
    cam.yaw                = yaw_rad;
    cam.pitch              = pitch_rad;

    cam.fov                = PI_VAL / 3.0f; // 60 graus
    cam.lock_fov           = false;

    // Planos de recorte ajustados ao tamanho da sala para preservar precisao
    // no Z-buffer e evitar z-fighting (secao 3.3 do plano).
    cam.near_plane         = -0.1f;
    cam.far_plane          = -25.0f;
    cam.mouse_sensitivity  = 0.0025f;

    cam.lookat_instance    = -1;
    cam.lookat_target      = glm::vec3(0.0f, 0.5f, 0.0f);
    cam.lookat_distance    = 2.2f;
    cam.lookat_theta       = 0.0f;
    cam.lookat_phi         = 0.30f;
    cam.lookat_orbit_speed = 1.60f;
}

void CameraEnterLookAt(Camera& cam, int instance_index, const glm::vec3& target_pos)
{
    cam.mode            = CAM_LOOK_AT;
    cam.lookat_instance = instance_index;
    cam.lookat_target   = target_pos;

    // Calcula os angulos esfericos iniciais a partir do vetor alvo -> olhos do
    // jogador, para que a transicao nao de um salto brusco de ponto de vista.
    glm::vec4 offset(cam.eye.x - target_pos.x,
                     cam.eye.y - target_pos.y,
                     cam.eye.z - target_pos.z,
                     0.0f);
    float d = norm(offset);
    if (d > 1e-4f)
    {
        float sin_phi = offset.y / d;
        if (sin_phi >  0.99f) sin_phi =  0.99f;
        if (sin_phi < -0.99f) sin_phi = -0.99f;

        cam.lookat_phi   = std::asin(sin_phi);
        cam.lookat_theta = std::atan2(offset.x, offset.z);

        if (cam.lookat_phi >  MAX_PITCH) cam.lookat_phi =  MAX_PITCH;
        if (cam.lookat_phi < -MAX_PITCH) cam.lookat_phi = -MAX_PITCH;

        cam.lookat_distance = std::min(std::max(d, 0.8f), 3.0f);
    }
}

void CameraUpdateLookAt(Camera& cam, const glm::vec3& target_pos,
                        bool orbit_left, bool orbit_right,
                        bool orbit_up, bool orbit_down,
                        float dt)
{
    cam.lookat_target = target_pos;

    if (dt <= 0.0f)
        return;

    float dtheta = 0.0f;
    float dphi   = 0.0f;
    if (orbit_left)  dtheta -= 1.0f;
    if (orbit_right) dtheta += 1.0f;
    if (orbit_up)    dphi   += 1.0f;
    if (orbit_down)  dphi   -= 1.0f;

    cam.lookat_theta += dtheta * cam.lookat_orbit_speed * dt;
    cam.lookat_phi   += dphi   * cam.lookat_orbit_speed * dt;

    if (cam.lookat_phi >  MAX_PITCH) cam.lookat_phi =  MAX_PITCH;
    if (cam.lookat_phi < -MAX_PITCH) cam.lookat_phi = -MAX_PITCH;
}

void CameraProcessMouse(Camera& cam, float dx, float dy)
{
    if (cam.mode == CAM_FIRST_PERSON)
    {
        // Em coordenadas de tela o eixo Y cresce para baixo: subir o mouse
        // produz dy < 0, entao subtraimos dy para que olhar para cima aumente
        // o pitch.
        cam.yaw   += dx * cam.mouse_sensitivity;
        cam.pitch -= dy * cam.mouse_sensitivity;

        if (cam.pitch >  MAX_PITCH) cam.pitch =  MAX_PITCH;
        if (cam.pitch < -MAX_PITCH) cam.pitch = -MAX_PITCH;
    }
    else
    {
        cam.lookat_theta -= dx * cam.mouse_sensitivity * 1.5f;
        cam.lookat_phi   += dy * cam.mouse_sensitivity * 1.5f;

        if (cam.lookat_phi >  MAX_PITCH) cam.lookat_phi =  MAX_PITCH;
        if (cam.lookat_phi < -MAX_PITCH) cam.lookat_phi = -MAX_PITCH;
    }
}

void CameraProcessScroll(Camera& cam, float yoffset)
{
    if (cam.mode == CAM_LOOK_AT)
    {
        cam.lookat_distance -= 0.25f * yoffset;
        if (cam.lookat_distance < 0.4f)
            cam.lookat_distance = 0.4f;
        if (cam.lookat_distance > 7.0f)
            cam.lookat_distance = 7.0f;
    }
    else if (!cam.lock_fov)
    {
        // Enquanto um objeto estiver na mao (lock_fov == true), o FOV nao pode
        // mudar para nao quebrar a invariante de tamanho aparente (secao 3.1).
        const float min_fov = PI_VAL / 6.0f; // 30 graus
        const float max_fov = PI_VAL * 0.45f; // 81 graus
        cam.fov -= 0.03f * yoffset;
        if (cam.fov < min_fov) cam.fov = min_fov;
        if (cam.fov > max_fov) cam.fov = max_fov;
    }
}

glm::vec4 CameraEyePosition(const Camera& cam)
{
    if (cam.mode == CAM_FIRST_PERSON)
        return glm::vec4(cam.eye.x, cam.eye.y, cam.eye.z, 1.0f);

    // Direcao radial unitaria do alvo ate a camera na esfera de orbita.
    float dir_y = std::sin(cam.lookat_phi);
    float dir_z = std::cos(cam.lookat_phi) * std::cos(cam.lookat_theta);
    float dir_x = std::cos(cam.lookat_phi) * std::sin(cam.lookat_theta);

    float r = cam.lookat_distance;

    // Limita o raio da orbita ao volume interno da sala (com folga maior que o
    // near plane) para que a camera look-at nunca atravesse paredes, piso ou
    // teto ao girar ao redor de um objeto proximo dos cantos. Como o recuo e
    // feito ao longo da propria direcao radial (dir_x, dir_y, dir_z), o vetor
    // de vista (target - eye) permanece exatamente na mesma direcao.
    const float margin = 0.18f;
    const float min_b[3] = { -Room::HALF_X + margin, margin,                -Room::HALF_Z + margin };
    const float max_b[3] = {  Room::HALF_X - margin, Room::HEIGHT - margin,  Room::HALF_Z - margin };
    const float tgt[3]   = { cam.lookat_target.x, cam.lookat_target.y, cam.lookat_target.z };
    const float dir[3]   = { dir_x, dir_y, dir_z };

    for (int i = 0; i < 3; ++i)
    {
        if (dir[i] > 1e-5f)
        {
            float r_lim = (max_b[i] - tgt[i]) / dir[i];
            if (r_lim >= 0.25f && r_lim < r)
                r = r_lim;
        }
        else if (dir[i] < -1e-5f)
        {
            float r_lim = (min_b[i] - tgt[i]) / dir[i];
            if (r_lim >= 0.25f && r_lim < r)
                r = r_lim;
        }
    }

    return glm::vec4(cam.lookat_target.x + r * dir_x,
                     cam.lookat_target.y + r * dir_y,
                     cam.lookat_target.z + r * dir_z,
                     1.0f);
}

glm::vec4 CameraForward(const Camera& cam)
{
    if (cam.mode == CAM_FIRST_PERSON)
    {
        // Coordenadas esfericas: yaw gira no plano XZ (0 aponta para +X,
        // +pi/2 para +Z) e pitch inclina em direcao a +Y.
        glm::vec4 v(std::cos(cam.pitch) * std::cos(cam.yaw),
                    std::sin(cam.pitch),
                    std::cos(cam.pitch) * std::sin(cam.yaw),
                    0.0f);
        return v / norm(v);
    }

    glm::vec4 eye    = CameraEyePosition(cam);
    glm::vec4 target = glm::vec4(cam.lookat_target.x,
                                 cam.lookat_target.y,
                                 cam.lookat_target.z,
                                 1.0f);
    glm::vec4 v = target - eye;
    return v / norm(v);
}

glm::vec4 CameraRight(const Camera& cam)
{
    glm::vec4 up_world(0.0f, 1.0f, 0.0f, 0.0f);
    glm::vec4 w = -CameraForward(cam);
    glm::vec4 u = crossproduct(up_world, w);
    return u / norm(u);
}

glm::vec4 CameraUp(const Camera& cam)
{
    glm::vec4 w = -CameraForward(cam);
    glm::vec4 u = CameraRight(cam);
    glm::vec4 v = crossproduct(w, u);
    return v / norm(v);
}

glm::mat4 CameraViewMatrix(const Camera& cam)
{
    glm::vec4 position_c  = CameraEyePosition(cam);
    glm::vec4 view_vector = CameraForward(cam);
    glm::vec4 up_vector   = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);

    return Matrix_Camera_View(position_c, view_vector, up_vector);
}

glm::mat4 CameraProjectionMatrix(const Camera& cam,
                                 float screen_ratio,
                                 bool perspective)
{
    if (perspective)
    {
        return Matrix_Perspective(cam.fov, screen_ratio,
                                  cam.near_plane, cam.far_plane);
    }

    float dist = (cam.mode == CAM_LOOK_AT) ? cam.lookat_distance : 3.5f;
    float t = 1.5f * dist / 2.5f;
    float b = -t;
    float r = t * screen_ratio;
    float l = -r;
    return Matrix_Orthographic(l, r, b, t, cam.near_plane, cam.far_plane);
}
