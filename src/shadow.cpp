#include "shadow.h"

#include <cstdio>
#include <cmath>

#include <glad/glad.h>

#include "matrices.h"

// Direcao de vista e vetor "up" de cada face do cube map, na convencao do
// OpenGL. O "up" invertido nas faces laterais nao e engano: e a orientacao
// que a especificacao de cube map exige para que a amostragem por vetor caia
// na face certa e na orientacao certa.
struct FaceBasis { glm::vec3 forward; glm::vec3 up; };

static const FaceBasis kFaces[6] = {
    { glm::vec3( 1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f) }, // +X
    { glm::vec3(-1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f) }, // -X
    { glm::vec3( 0.0f,  1.0f,  0.0f), glm::vec3(0.0f,  0.0f,  1.0f) }, // +Y
    { glm::vec3( 0.0f, -1.0f,  0.0f), glm::vec3(0.0f,  0.0f, -1.0f) }, // -Y
    { glm::vec3( 0.0f,  0.0f,  1.0f), glm::vec3(0.0f, -1.0f,  0.0f) }, // +Z
    { glm::vec3( 0.0f,  0.0f, -1.0f), glm::vec3(0.0f, -1.0f,  0.0f) }  // -Z
};

bool PointShadowInit(PointShadow& s, int resolution, int texture_unit,
                     const glm::vec3& position, float far_range)
{
    s.fbo          = 0;
    s.cube_texture = 0;
    s.depth_rbo    = 0;
    s.resolution   = resolution;
    s.texture_unit = texture_unit;
    s.ready        = false;
    s.position     = position;
    s.far_range    = far_range;

    // Escolhemos a unidade ANTES de criar a textura: glBindTexture age sobre
    // a unidade ativa, e deixar isso ao acaso sobrescreve a textura de quem
    // tiver sido carregado por ultimo.
    glActiveTexture(GL_TEXTURE0 + texture_unit);

    glGenTextures(1, &s.cube_texture);
    glBindTexture(GL_TEXTURE_CUBE_MAP, s.cube_texture);

    // Um canal float por texel: a distancia normalizada ate a luz.
    for (int face = 0; face < 6; ++face)
    {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, 0, GL_R32F,
                     resolution, resolution, 0, GL_RED, GL_FLOAT, NULL);
    }

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    glActiveTexture(GL_TEXTURE0);

    // z-buffer auxiliar, para que o passo de profundidade resolva quem esta
    // na frente dentro de cada face. E compartilhado: cada face e limpa antes
    // de ser desenhada.
    glGenRenderbuffers(1, &s.depth_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, s.depth_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24,
                          resolution, resolution);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    glGenFramebuffers(1, &s.fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, s.fbo);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, s.depth_rbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_CUBE_MAP_POSITIVE_X, s.cube_texture, 0);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
        fprintf(stderr,
                "AVISO: framebuffer de sombra incompleto (0x%x). "
                "O jogo seguira sem sombra desta luz.\n", status);
        PointShadowDestroy(s);
        return false;
    }

    s.ready = true;
    return true;
}

glm::mat4 PointShadowFaceMatrix(const PointShadow& s, int face)
{
    if (face < 0 || face > 5)
        face = 0;

    glm::vec4 pos(s.position.x, s.position.y, s.position.z, 1.0f);
    glm::vec4 fwd(kFaces[face].forward.x, kFaces[face].forward.y,
                  kFaces[face].forward.z, 0.0f);
    glm::vec4 up(kFaces[face].up.x, kFaces[face].up.y, kFaces[face].up.z, 0.0f);

    glm::mat4 view = Matrix_Camera_View(pos, fwd, up);

    // Cada face cobre exatamente 90 graus, e e quadrada.
    const float ninety = 1.57079632679489661923f;
    glm::mat4 proj = Matrix_Perspective(ninety, 1.0f,
                                        ShadowConfig::NEAR_PLANE,
                                        -s.far_range);

    return proj * view;
}

void PointShadowBeginFace(const PointShadow& s, int face)
{
    glBindFramebuffer(GL_FRAMEBUFFER, s.fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_CUBE_MAP_POSITIVE_X + face,
                           s.cube_texture, 0);
    glViewport(0, 0, s.resolution, s.resolution);

    // Limpamos a cor com 1.0 = alcance maximo: onde nao houver geometria, a
    // leitura dira "nada bloqueia a luz".
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void PointShadowEnd(int framebuffer_width, int framebuffer_height)
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, framebuffer_width, framebuffer_height);
}

void PointShadowBindForReading(const PointShadow& s)
{
    glActiveTexture(GL_TEXTURE0 + s.texture_unit);
    glBindTexture(GL_TEXTURE_CUBE_MAP, s.cube_texture);
    glBindSampler(s.texture_unit, 0);
    glActiveTexture(GL_TEXTURE0);
}

void PointShadowDestroy(PointShadow& s)
{
    if (s.cube_texture != 0) { glDeleteTextures(1, &s.cube_texture);      s.cube_texture = 0; }
    if (s.depth_rbo    != 0) { glDeleteRenderbuffers(1, &s.depth_rbo);    s.depth_rbo    = 0; }
    if (s.fbo          != 0) { glDeleteFramebuffers(1, &s.fbo);           s.fbo          = 0; }
    s.ready = false;
}
