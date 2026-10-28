#include "camera.h"
#include "matrices.h"
#include "scene.h"

#include <cmath>

static const float PI_VAL    = 3.14159265f;
static const float MAX_PITCH = 1.50f; // ~86 graus: evita view_vector colinear com up (0,1,0)

void CameraInit(Camera& cam, const glm::vec3& eye_pos,
                float yaw_rad, float pitch_rad)
{
    cam.mode              = CAM_FIRST_PERSON;
    cam.eye               = eye_pos;
    cam.yaw               = yaw_rad;
    cam.pitch             = pitch_rad;

    cam.fov               = PI_VAL / 3.0f; // 60 graus
    cam.lock_fov          = false;

    // Planos de recorte ajustados ao tamanho da sala (8 x 6 x 3 m) para
    // preservar precisao no Z-buffer e evitar z-fighting (secao 3.3 do plano).
    cam.near_plane        = -0.1f;
    cam.far_plane         = -25.0f;
    cam.mouse_sensitivity = 0.0025f;

    cam.lookat_target     = glm::vec3(0.0f, Room::HEIGHT * 0.5f, 0.0f);
    cam.lookat_distance   = 3.5f;
    cam.lookat_theta      = 0.0f;
    cam.lookat_phi        = 0.25f;
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
        cam.lookat_theta -= dx * 0.01f;
        cam.lookat_phi   += dy * 0.01f;

        if (cam.lookat_phi >  MAX_PITCH) cam.lookat_phi =  MAX_PITCH;
        if (cam.lookat_phi < -MAX_PITCH) cam.lookat_phi = -MAX_PITCH;
    }
}

void CameraProcessScroll(Camera& cam, float yoffset)
{
    if (cam.mode == CAM_LOOK_AT)
    {
        cam.lookat_distance -= 0.2f * yoffset;
        if (cam.lookat_distance < 0.3f)
            cam.lookat_distance = 0.3f;
        if (cam.lookat_distance > 15.0f)
            cam.lookat_distance = 15.0f;
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

    float r = cam.lookat_distance;
    float y = r * std::sin(cam.lookat_phi);
    float z = r * std::cos(cam.lookat_phi) * std::cos(cam.lookat_theta);
    float x = r * std::cos(cam.lookat_phi) * std::sin(cam.lookat_theta);

    return glm::vec4(cam.lookat_target.x + x,
                     cam.lookat_target.y + y,
                     cam.lookat_target.z + z,
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
