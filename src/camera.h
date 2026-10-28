#ifndef CAMERA_H
#define CAMERA_H

// Modulo de camera: suporta a camera livre em primeira pessoa (controlada por
// yaw/pitch no mouse e presa aos olhos do jogador) e a camera look-at (que
// orbita um ponto de interesse, requisito de "duas cameras" do enunciado).
//
// Todas as matrizes (View e Projection) e operacoes vetoriais (norma, produto
// vetorial) usam exclusivamente "matrices.h", sem chamar funcoes prontas da
// GLM.

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>

enum CameraMode
{
    CAM_FIRST_PERSON = 0, // camera livre em 1a pessoa presa aos olhos do jogador
    CAM_LOOK_AT      = 1  // camera look-at orbitando um alvo
};

struct Camera
{
    CameraMode mode;

    // --- Estado da camera em 1a pessoa ---
    glm::vec3 eye;               // posicao dos olhos em coordenadas de mundo
    float     yaw;               // rotacao no plano XZ, em radianos (0 aponta para +X)
    float     pitch;             // inclinacao vertical, em radianos, presa em (-pi/2, +pi/2)

    // --- Parametros opticos e de controle ---
    float     fov;               // abertura vertical em radianos
    bool      lock_fov;          // trava o FOV enquanto um objeto esta sendo carregado (M2)
    float     near_plane;        // plano near (negativo na convencao de Matrix_Perspective)
    float     far_plane;         // plano far  (negativo, ajustado ao tamanho da sala)
    float     mouse_sensitivity; // radianos por pixel de deslocamento do cursor

    // --- Estado da camera look-at ---
    glm::vec3 lookat_target;     // ponto para onde a camera look-at olha
    float     lookat_distance;   // raio da orbita em metros
    float     lookat_theta;      // angulo horizontal da orbita
    float     lookat_phi;        // angulo vertical da orbita
};

// Inicializa a camera em 1a pessoa na posicao e orientacao dadas.
void CameraInit(Camera& cam, const glm::vec3& eye_pos,
                float yaw_rad = 0.0f, float pitch_rad = 0.0f);

// Atualiza a orientacao da camera a partir do deslocamento (dx, dy) do mouse
// em pixels. Na 1a pessoa altera yaw/pitch; na look-at orbita theta/phi.
void CameraProcessMouse(Camera& cam, float dx, float dy);

// Atualiza o zoom/distancia pela rodinha do mouse (respeitando lock_fov).
void CameraProcessScroll(Camera& cam, float yoffset);

// Posicao efetiva da camera em mundo (w = 1) no modo ativo.
glm::vec4 CameraEyePosition(const Camera& cam);

// Vetor de vista (direcao para onde a camera aponta, w = 0, normalizado).
glm::vec4 CameraForward(const Camera& cam);

// Vetores da base ortonormal da camera em mundo (w = 0, normalizados).
// Sao os mesmos vetores u (right) e v (up) construidos em Matrix_Camera_View,
// expostos aqui para construir o raio do cursor sem inverter matriz (M2).
glm::vec4 CameraRight(const Camera& cam);
glm::vec4 CameraUp(const Camera& cam);

// Matriz View calculada via Matrix_Camera_View (matrices.h).
glm::mat4 CameraViewMatrix(const Camera& cam);

// Matriz Projection calculada via Matrix_Perspective ou Matrix_Orthographic
// (matrices.h).
glm::mat4 CameraProjectionMatrix(const Camera& cam,
                                 float screen_ratio,
                                 bool perspective = true);

#endif // CAMERA_H
