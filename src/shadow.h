#ifndef SHADOW_H
#define SHADOW_H

// Sombras onidirecionais para fontes de luz PONTUAIS (shadow cube map).
//
// Por que cube map, e nao um mapa unico:
//
//   Um mapa de sombra comum guarda a profundidade vista de UMA direcao. Isso
//   funciona para luz direcional, mas nao para uma lampada: a sombra de um
//   objeto deve sair RADIALMENTE da fonte, mudando de direcao conforme o
//   objeto se desloca em relacao a ela. Com uma direcao fixa, a sombra cai
//   sempre para o mesmo lado, e um objeto no canto da sala projeta como se a
//   luz estivesse em cima dele.
//
//   Tambem nao adianta usar uma projecao perspectiva unica apontada para
//   baixo: um objeto erguido ate perto do teto, longe do eixo da lampada,
//   fica a quase 90 graus dela e cairia fora de qualquer cone possivel.
//
//   O cube map resolve os dois casos: sao seis renderizacoes de 90 graus, uma
//   por face, cobrindo todas as direcoes ao redor da luz.
//
// O que gravamos em cada face NAO e a profundidade do z-buffer, e sim a
// DISTANCIA EUCLIDIANA da luz ate a superficie, normalizada pelo alcance. Com
// isso a comparacao no shader principal fica direta: basta medir a distancia
// do fragmento ate a luz e confrontar com o valor lido.
//
// As matrizes View e Projection de cada face sao construidas com
// Matrix_Camera_View e Matrix_Perspective de "matrices.h", as nossas.

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

namespace ShadowConfig
{
    const int   RESOLUTION = 512;   // lado de cada uma das seis faces
    const float FAR_RANGE  = 20.0f; // alcance da luz, em metros
    // Near bem pequeno de proposito. Com a mecanica do jogo o jogador pode
    // encostar um objeto na propria luminaria, e qualquer parte dele mais
    // proxima que o near seria recortada e deixaria de projetar sombra.
    // Aqui isso custa pouca precisao, porque o mapa guarda DISTANCIA numa
    // textura float; o z-buffer so ordena os objetos entre si.
    const float NEAR_PLANE = -0.02f;

    // Unidades de textura dos dois cube maps. As imagens da cena ocupam
    // 0, 1, 2...; deixamos folga.
    const int TEXTURE_UNIT_1 = 5;
    const int TEXTURE_UNIT_2 = 6;
}

struct PointShadow
{
    unsigned int fbo;
    unsigned int cube_texture;  // GL_TEXTURE_CUBE_MAP, um canal float
    unsigned int depth_rbo;     // z-buffer auxiliar, compartilhado pelas faces
    int          resolution;
    int          texture_unit;
    bool         ready;

    glm::vec3 position;         // posicao da luz em mundo
    float     far_range;        // distancia maxima gravada
};

bool PointShadowInit(PointShadow& s, int resolution, int texture_unit,
                     const glm::vec3& position, float far_range);

// Matriz Projection * View da face "face" (0..5), na ordem das faces de
// cube map do OpenGL: +X, -X, +Y, -Y, +Z, -Z.
glm::mat4 PointShadowFaceMatrix(const PointShadow& s, int face);

// Liga o framebuffer apontando para a face dada e limpa. Tudo desenhado
// depois disto vai para aquela face.
void PointShadowBeginFace(const PointShadow& s, int face);

// Volta a desenhar na janela.
void PointShadowEnd(int framebuffer_width, int framebuffer_height);

void PointShadowBindForReading(const PointShadow& s);

void PointShadowDestroy(PointShadow& s);

#endif // SHADOW_H
