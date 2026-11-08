#version 330 core

// Atributos de vértice recebidos como entrada ("in") pelo Vertex Shader.
// Veja a função BuildTrianglesAndAddToVirtualScene() em "main.cpp".
layout (location = 0) in vec4 model_coefficients;
layout (location = 1) in vec4 normal_coefficients;
layout (location = 2) in vec2 texture_coefficients;

// Matrizes computadas no código C++ e enviadas para a GPU
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

// Modelo de interpolacao de iluminacao (enum ShadingMode de scene.h):
//   0 = SHADING_PHONG   (iluminacao avaliada por fragmento no fragment shader)
//   1 = SHADING_GOURAUD (iluminacao avaliada por vertice aqui no vertex shader
//                        e interpolada pelo rasterizador)
#define SHADING_PHONG   0
#define SHADING_GOURAUD 1

uniform int   shading_mode;
uniform float ks;
uniform float shininess;

// Mesmos parametros das duas fontes de luz e do ambiente declarados no
// fragment shader, compartilhando as localizacoes de uniform no programa.
uniform vec4  light1_pos;
uniform vec3  light1_color;
uniform vec4  light2_pos;
uniform vec3  light2_color;
uniform vec4  light2_dir;
uniform float light2_cos_outer;
uniform float light2_cos_inner;
uniform vec3  ambient_sky;
uniform vec3  ambient_ground;
uniform vec4  camera_position;

// Atributos de vértice que serão gerados como saída ("out") pelo Vertex Shader.
// ** Estes serão interpolados pelo rasterizador! ** gerando, assim, valores
// para cada fragmento, os quais serão recebidos como entrada pelo Fragment
// Shader. Veja o arquivo "shader_fragment.glsl".
out vec4 position_world;
out vec4 position_model;
out vec4 normal;
out vec2 texcoords;

// Termos de iluminacao calculados por vertice para interpolacao de Gouraud.
out vec3  gouraud_ambient;
out vec3  gouraud_diff1;
out vec3  gouraud_diff2;
out vec3  gouraud_spec1;
out vec3  gouraud_spec2;
out float gouraud_ndotl1;
out float gouraud_ndotl2;

void main()
{
    // A variável gl_Position define a posição final de cada vértice
    // OBRIGATORIAMENTE em "normalized device coordinates" (NDC), onde cada
    // coeficiente estará entre -1 e 1 após divisão por w.
    gl_Position = projection * view * model * model_coefficients;

    // Posição do vértice atual no sistema de coordenadas global (World).
    position_world = model * model_coefficients;

    // Posição do vértice atual no sistema de coordenadas local do modelo.
    position_model = model_coefficients;

    // Normal do vértice atual no sistema de coordenadas global (World).
    // Veja slides 123-151 do documento Aula_07_Transformacoes_Geometricas_3D.pdf.
    normal = inverse(transpose(model)) * normal_coefficients;
    normal.w = 0.0;

    // Coordenadas de textura obtidas do arquivo OBJ (se existirem!)
    texcoords = texture_coefficients;

    // -----------------------------------------------------------------------
    // Interpolacao de Gouraud (M5 - Assets e luz):
    // Quando shading_mode == SHADING_GOURAUD, o modelo de iluminacao de
    // Blinn-Phong (difuso de Lambert + especular (N.H)^n + ambiente
    // hemisferico) e avaliado aqui em cada VERTICE e interpolado pelo
    // rasterizador ate o fragment shader.
    // -----------------------------------------------------------------------
    if (shading_mode == SHADING_GOURAUD)
    {
        vec4 p = position_world;
        vec4 n = normalize(normal);
        vec4 v = normalize(camera_position - p);

        // Luz 1: faixa central no teto (pontual)
        vec3  d1v    = light1_pos.xyz - p.xyz;
        float d1     = max(length(d1v), 1e-6);
        vec4  l1     = vec4(d1v / d1, 0.0);
        vec4  h1     = normalize(l1 + v);
        float lamb1  = max(0.0, dot(n, l1));
        float blinn1 = (lamb1 > 0.0) ? pow(max(0.0, dot(n, h1)), shininess) : 0.0;
        float att1   = 1.0 / (1.0 + 0.10*d1 + 0.025*d1*d1);

        // Luz 2: pendente em cone na quina (spotlight)
        vec3  d2v    = light2_pos.xyz - p.xyz;
        float d2     = max(length(d2v), 1e-6);
        vec4  l2     = vec4(d2v / d2, 0.0);
        vec4  h2     = normalize(l2 + v);
        float lamb2  = max(0.0, dot(n, l2));
        float blinn2 = (lamb2 > 0.0) ? pow(max(0.0, dot(n, h2)), shininess) : 0.0;
        float att2   = 1.0 / (1.0 + 0.25*d2 + 0.08*d2*d2);

        float spot_cos = dot(-l2.xyz, normalize(light2_dir.xyz));
        float spot     = smoothstep(light2_cos_outer, light2_cos_inner, spot_cos);
        att2 *= spot;

        float up_amount = 0.5 + 0.5 * n.y;

        gouraud_ambient = mix(ambient_ground, ambient_sky, up_amount);
        gouraud_diff1   = lamb1 * att1 * light1_color;
        gouraud_diff2   = lamb2 * att2 * light2_color;
        gouraud_spec1   = vec3(ks) * blinn1 * att1 * light1_color;
        gouraud_spec2   = vec3(ks) * blinn2 * att2 * light2_color;
        gouraud_ndotl1  = lamb1;
        gouraud_ndotl2  = lamb2;
    }
    else
    {
        gouraud_ambient = vec3(0.0);
        gouraud_diff1   = vec3(0.0);
        gouraud_diff2   = vec3(0.0);
        gouraud_spec1   = vec3(0.0);
        gouraud_spec2   = vec3(0.0);
        gouraud_ndotl1  = 0.0;
        gouraud_ndotl2  = 0.0;
    }
}
