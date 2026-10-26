#version 330 core

// Atributos interpolados pelo rasterizador, vindos do vertex shader.
in vec4 position_world;
in vec4 position_model;
in vec4 normal;
in vec2 texcoords;

// Matrizes da aplicacao.
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

// Bounding box da MALHA em coordenadas locais (usada nas projecoes de textura
// que trabalham em coordenadas do modelo).
uniform vec4 bbox_min;
uniform vec4 bbox_max;

// ---------------------------------------------------------------------------
// Material da instancia sendo desenhada. Substitui o antigo "object_id" com
// if/else fixo, que nao escalava para a quantidade de objetos da fase.
// Os valores de uv_mode precisam bater com o enum UVMode de "scene.h".
// ---------------------------------------------------------------------------
#define UV_OBJ       0
#define UV_WORLD     1
#define UV_SPHERE    2
#define UV_PLANAR_XY 3

uniform int   uv_mode;
uniform float uv_scale;   // metros de mundo por repeticao (so UV_WORLD)
uniform int   tex_id;     // qual TextureImageN usar
uniform float ks;         // intensidade do especular
uniform float shininess;  // expoente do especular

// Fonte de luz e camera.
uniform vec4 light_position;
uniform vec3 light_color;
uniform vec4 camera_position;

uniform sampler2D TextureImage0;
uniform sampler2D TextureImage1;
uniform sampler2D TextureImage2;

out vec4 color;

#define M_PI   3.14159265358979323846
#define M_PI_2 1.57079632679489661923

// Le a textura escolhida pelo material. Em GLSL 3.30 um array de samplers nao
// pode ser indexado por variavel, entao a escolha e feita com if/else.
vec3 SampleTexture(vec2 uv)
{
    if (tex_id == 0) return texture(TextureImage0, uv).rgb;
    if (tex_id == 1) return texture(TextureImage1, uv).rgb;
    return texture(TextureImage2, uv).rgb;
}

void main()
{
    vec4 p = position_world;
    vec4 n = normalize(normal);

    vec2 uv;

    if (uv_mode == UV_WORLD)
    {
        // Projecao planar em COORDENADAS DE MUNDO, escolhendo o plano pelo
        // eixo dominante da normal.
        //
        // Este e o ponto central contra o desconto por "textura esticada": a
        // coordenada e medida em METROS e dividida por uv_scale, entao o texel
        // sai quadrado em qualquer superficie, nao importa o quanto a caixa
        // tenha sido escalada em cada eixo. Uma parede de 8 m e uma de 0.2 m
        // ficam com a textura no mesmo tamanho aparente.
        vec3 an = abs(n.xyz);

        if (an.y >= an.x && an.y >= an.z)
            uv = vec2(p.x, p.z);   // piso e teto
        else if (an.x >= an.z)
            uv = vec2(p.z, p.y);   // paredes perpendiculares a X
        else
            uv = vec2(p.x, p.y);   // paredes perpendiculares a Z

        uv /= uv_scale;
    }
    else if (uv_mode == UV_SPHERE)
    {
        // Projecao esferica em coordenadas do modelo, centrada na bbox.
        vec4 c = (bbox_min + bbox_max) / 2.0;
        vec4 d = position_model - c;

        float rho   = length(d);
        float theta = atan(d.x, d.z);
        float phi   = asin(d.y / rho);

        uv = vec2((theta + M_PI) / (2.0*M_PI),
                  (phi + M_PI_2) / M_PI);
    }
    else if (uv_mode == UV_PLANAR_XY)
    {
        // Projecao planar XY em coordenadas do modelo, normalizada pela bbox.
        uv = vec2((position_model.x - bbox_min.x) / (bbox_max.x - bbox_min.x),
                  (position_model.y - bbox_min.y) / (bbox_max.y - bbox_min.y));
    }
    else
    {
        // Coordenadas vindas do proprio arquivo OBJ.
        uv = texcoords;
    }

    vec3 Kd = SampleTexture(uv);

    // -----------------------------------------------------------------------
    // Iluminacao de Blinn-Phong, calculada POR FRAGMENTO (interpolacao de
    // Phong). O termo especular usa o vetor halfway h = normalize(l + v), que
    // e mais barato que refletir o vetor e se comporta melhor em angulos
    // rasantes do que o Phong classico.
    // -----------------------------------------------------------------------
    vec4 l = normalize(light_position - p);
    vec4 v = normalize(camera_position - p);
    vec4 h = normalize(l + v);

    float lambert = max(0.0, dot(n, l));
    float blinn   = pow(max(0.0, dot(n, h)), shininess);

    // Atenuacao pela distancia ate a fonte.
    float dist = length(light_position - p);
    float att  = 1.0 / (1.0 + 0.10*dist + 0.015*dist*dist);

    vec3 ambient  = 0.14 * Kd;
    vec3 diffuse  = Kd * lambert * att * light_color;
    vec3 specular = vec3(ks) * blinn * att * light_color;

    // O especular so faz sentido onde a luz de fato chega.
    if (lambert <= 0.0)
        specular = vec3(0.0);

    color.rgb = ambient + diffuse + specular;
    color.a   = 1.0;

    // Correcao gamma, considerando monitor sRGB.
    color.rgb = pow(color.rgb, vec3(1.0)/2.2);
}
