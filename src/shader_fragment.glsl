#version 330 core

in vec4 position_world;
in vec4 position_model;
in vec4 normal;
in vec2 texcoords;

// Termos interpolados a partir dos vertices quando shading_mode == SHADING_GOURAUD.
in vec3  gouraud_ambient;
in vec3  gouraud_diff1;
in vec3  gouraud_diff2;
in vec3  gouraud_spec1;
in vec3  gouraud_spec2;
in float gouraud_ndotl1;
in float gouraud_ndotl2;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform vec4 bbox_min;
uniform vec4 bbox_max;

// --- Como a coordenada de textura e obtida (enum UVMode de scene.h) ---
#define UV_OBJ       0
#define UV_WORLD     1
#define UV_SPHERE    2
#define UV_PLANAR_XY 3

// --- Padrao procedural que gera a cor (enum PatternMode de scene.h) ---
#define PATTERN_TEXTURE 0
#define PATTERN_WALL    1
#define PATTERN_FLOOR   2
#define PATTERN_CEILING 3

// --- Modelo de interpolacao de iluminacao (enum ShadingMode de scene.h) ---
#define SHADING_PHONG   0
#define SHADING_GOURAUD 1

uniform int   uv_mode;
uniform float uv_scale;   // metros de mundo por repeticao (so UV_WORLD)
uniform int   tex_id;
uniform float ks;
uniform float shininess;
uniform int   pattern;
uniform float emissive;   // > 0: objeto brilha com cor propria (luminaria)
uniform int   shading_mode;

// As duas luminarias da fase, ambas tratadas como fontes PONTUAIS: a faixa
// curta no centro do teto e o pendente em cone na quina. Cada uma tem o seu
// proprio cube map de sombra, entao sombreamento e sombra usam exatamente a
// mesma posicao e nunca discordam.
uniform vec4 light1_pos;
uniform vec3 light1_color;
uniform vec4 light2_pos;
uniform vec3 light2_color;

// O pendente e um SPOTLIGHT, nao uma fonte pontual: a cupula em cone so
// deixa a luz sair para baixo, dentro de uma abertura.
uniform vec4  light2_dir;        // direcao do eixo do cone, normalizada
uniform float light2_cos_outer;  // cosseno da abertura
uniform float light2_cos_inner;  // cosseno de onde comeca a suavizar a borda

// Ambiente hemisferico: componente que chega de cima e componente que chega
// de baixo, interpoladas pela inclinacao da normal.
uniform vec3 ambient_sky;
uniform vec3 ambient_ground;

uniform vec4 camera_position;

uniform sampler2D TextureImage0;
uniform sampler2D TextureImage1;
uniform sampler2D TextureImage2;
uniform sampler2D TextureImage3;
uniform sampler2D TextureImage4;

// --- Shadow mapping onidirecional ---
//
// Um cube map por luz, guardando a DISTANCIA ate a primeira superficie em
// cada direcao, normalizada por shadow_far.
uniform samplerCube ShadowCube1;
uniform samplerCube ShadowCube2;
uniform float       shadow_far;
uniform int         shadow_enabled;

out vec4 color;

#define M_PI   3.14159265358979323846
#define M_PI_2 1.57079632679489661923

// Em GLSL 3.30 um array de samplers nao pode ser indexado por variavel.
vec3 SampleTexture(vec2 uv)
{
    if (tex_id == 0) return texture(TextureImage0, uv).rgb;
    if (tex_id == 1) return texture(TextureImage1, uv).rgb;
    if (tex_id == 2) return texture(TextureImage2, uv).rgb;
    if (tex_id == 3) return texture(TextureImage3, uv).rgb;
    return texture(TextureImage4, uv).rgb;
}

// ---------------------------------------------------------------------------
// Texturas procedurais.
//
// O SPEC.md especifica que o cenario seria feito de composicoes procedurais
// em vez de imagens. A vantagem pratica: a cor e gerada a partir da posicao em
// METROS no mundo, entao paineis, juntas e grade tem sempre o mesmo tamanho
// real, em qualquer parede, independente de como a caixa foi escalada. Nao
// existe como "esticar" essas texturas.
// ---------------------------------------------------------------------------

// Distancia ate a linha de grade mais proxima, para um periodo dado.
float GridDistance(float x, float period)
{
    float q = fract(x / period);
    return min(q, 1.0 - q) * period;
}

// Parede: paineis creme separados por juntas verticais escuras, com rodape
// verde na base, como na imagem de referencia do SPEC.
vec3 WallPattern(vec4 p, vec2 uv)
{
    const vec3 CREAM = vec3(0.855, 0.790, 0.620);
    const vec3 SEAM  = vec3(0.250, 0.235, 0.205);
    const vec3 SKIRT = vec3(0.145, 0.345, 0.250);

    const float PANEL_WIDTH = 1.6;  // metros
    const float SKIRT_H     = 0.55; // altura do rodape, em metros

    // Juntas verticais entre paineis. uv.x e a coordenada horizontal em
    // metros, vinda da projecao UV_WORLD.
    float d    = GridDistance(uv.x, PANEL_WIDTH);
    float seam = 1.0 - smoothstep(0.012, 0.030, d);

    // Gradiente vertical suave, para a parede nao ficar completamente chapada.
    float grad = clamp(p.y / 3.2, 0.0, 1.0);
    vec3  col  = mix(CREAM*0.90, CREAM, grad);

    col = mix(col, SEAM, seam);

    // Rodape e a linha escura que marca o topo dele.
    float in_skirt = 1.0 - smoothstep(SKIRT_H, SKIRT_H + 0.015, p.y);
    col = mix(col, SKIRT, in_skirt);

    float lip = 1.0 - smoothstep(0.010, 0.026, abs(p.y - SKIRT_H));
    col = mix(col, SKIRT*0.55, lip);

    return col;
}

// Piso: verde, com grade de 1 m e um xadrez muito sutil. A grade e o que da
// ao jogador a referencia de escala e de profundidade no chao.
vec3 FloorPattern(vec2 uv)
{
    const vec3 GREEN_A = vec3(0.170, 0.400, 0.302);
    const vec3 GREEN_B = vec3(0.145, 0.352, 0.268);
    const vec3 LINE    = vec3(0.098, 0.252, 0.196);

    const float TILE = 1.0;

    float d    = min(GridDistance(uv.x, TILE), GridDistance(uv.y, TILE));
    float line = 1.0 - smoothstep(0.012, 0.028, d);

    float checker = mod(floor(uv.x/TILE) + floor(uv.y/TILE), 2.0);
    vec3  base    = mix(GREEN_A, GREEN_B, checker);

    return mix(base, LINE, line);
}

// Teto: claro, dividido em paineis maiores.
vec3 CeilingPattern(vec2 uv)
{
    const vec3 WHITE = vec3(0.960, 0.955, 0.930);
    const vec3 LINE  = vec3(0.660, 0.660, 0.645);

    const float TILE = 1.2;

    float d    = min(GridDistance(uv.x, TILE), GridDistance(uv.y, TILE));
    float line = 1.0 - smoothstep(0.010, 0.024, d);

    return mix(WHITE, LINE, line);
}

// ---------------------------------------------------------------------------
// Fator de sombra em [0, 1] para uma luz pontual: 0 totalmente iluminado,
// 1 totalmente na sombra.
// ---------------------------------------------------------------------------
float PointShadowFactor(samplerCube cube, vec3 light_pos,
                        vec4 p_world, vec4 n_world, float n_dot_l)
{
    if (shadow_enabled == 0)
        return 0.0;

    if (n_dot_l <= 0.0)
        return 0.0;

    // NORMAL OFFSET BIAS: afastamos o ponto de amostragem ao longo da normal
    // antes de consultar o mapa.
    float offset = min(0.012 + 0.045 * (1.0 - n_dot_l), 0.035);
    vec3  sp     = p_world.xyz + n_world.xyz * offset;

    vec3  dir  = sp - light_pos;
    float dist = length(dir);

    if (dist >= shadow_far)
        return 0.0;

    float bias = min(0.020 + 0.120 * (1.0 - n_dot_l), 0.060);

    const vec3 K[8] = vec3[8](
        vec3( 1.0,  1.0,  1.0), vec3( 1.0, -1.0,  1.0),
        vec3(-1.0, -1.0,  1.0), vec3(-1.0,  1.0,  1.0),
        vec3( 1.0,  1.0, -1.0), vec3( 1.0, -1.0, -1.0),
        vec3(-1.0, -1.0, -1.0), vec3(-1.0,  1.0, -1.0));

    float radius = min(dist * 0.006, 0.05);
    float sum    = (dist - bias > texture(cube, dir).r * shadow_far) ? 1.0 : 0.0;

    for (int i = 0; i < 8; ++i)
    {
        float st = texture(cube, dir + K[i]*radius).r * shadow_far;
        sum += (dist - bias > st) ? 1.0 : 0.0;
    }

    return sum / 9.0;
}

void main()
{
    // Objeto emissivo (a luminaria do teto): tem cor propria e nao participa
    // da iluminacao nem da sombra. E o que o jogador le como fonte de luz.
    if (emissive > 0.0)
    {
        color.rgb = pow(vec3(1.0, 0.97, 0.90) * emissive, vec3(1.0)/2.2);
        color.a   = 1.0;
        return;
    }

    vec4 p = position_world;
    vec4 n = normalize(normal);

    // --- coordenada de textura ---
    vec2 uv;

    if (uv_mode == UV_WORLD)
    {
        // Projecao planar em COORDENADAS DE MUNDO pelo eixo dominante da
        // normal. A coordenada sai em metros, entao o padrao tem o mesmo
        // tamanho real em qualquer superficie: e isto que impede textura
        // esticada, que e desconto explicito nos criterios.
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
        uv = vec2((position_model.x - bbox_min.x) / (bbox_max.x - bbox_min.x),
                  (position_model.y - bbox_min.y) / (bbox_max.y - bbox_min.y));
    }
    else
    {
        uv = texcoords;
    }

    // --- refletancia difusa: imagem ou padrao procedural ---
    vec3 Kd;

    if (pattern == PATTERN_WALL)
        Kd = WallPattern(p, uv);
    else if (pattern == PATTERN_FLOOR)
        Kd = FloorPattern(uv);
    else if (pattern == PATTERN_CEILING)
        Kd = CeilingPattern(uv);
    else
        Kd = SampleTexture(uv);

    // -----------------------------------------------------------------------
    // Iluminacao:
    //   - SHADING_GOURAUD (1): usa os termos difuso, especular e ambiente ja
    //     calculados por vertice em shader_vertex.glsl e interpolados pelo
    //     rasterizador, modulados por Kd e pelas sombras das duas luzes.
    //   - SHADING_PHONG   (0): avalia Blinn-Phong por fragmento usando a
    //     normal interpolada n e o meio-vetor h = normalize(l + v).
    // -----------------------------------------------------------------------
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;

    if (shading_mode == SHADING_GOURAUD)
    {
        float shad1 = PointShadowFactor(ShadowCube1, light1_pos.xyz, p, n, gouraud_ndotl1);
        float shad2 = PointShadowFactor(ShadowCube2, light2_pos.xyz, p, n, gouraud_ndotl2);

        ambient  = Kd * gouraud_ambient;
        diffuse  = Kd * gouraud_diff1 * (1.0 - shad1)
                 + Kd * gouraud_diff2 * (1.0 - shad2);
        specular = gouraud_spec1 * (1.0 - shad1)
                 + gouraud_spec2 * (1.0 - shad2);
    }
    else
    {
        vec4 v = normalize(camera_position - p);

        // ----- luz 1: faixa curta no centro do teto -----
        vec3  d1v    = light1_pos.xyz - p.xyz;
        float d1     = max(length(d1v), 1e-6);
        vec4  l1     = vec4(d1v / d1, 0.0);
        vec4  h1     = normalize(l1 + v);
        float lamb1  = max(0.0, dot(n, l1));
        float blinn1 = pow(max(0.0, dot(n, h1)), shininess);
        float att1   = 1.0 / (1.0 + 0.10*d1 + 0.025*d1*d1);
        float shad1  = PointShadowFactor(ShadowCube1, light1_pos.xyz, p, n, lamb1);

        // ----- luz 2: pendente em cone na quina -----
        vec3  d2v    = light2_pos.xyz - p.xyz;
        float d2     = max(length(d2v), 1e-6);
        vec4  l2     = vec4(d2v / d2, 0.0);
        vec4  h2     = normalize(l2 + v);
        float lamb2  = max(0.0, dot(n, l2));
        float blinn2 = pow(max(0.0, dot(n, h2)), shininess);
        float att2   = 1.0 / (1.0 + 0.25*d2 + 0.08*d2*d2);

        float spot_cos = dot(-l2.xyz, normalize(light2_dir.xyz));
        float spot     = smoothstep(light2_cos_outer, light2_cos_inner, spot_cos);
        att2 *= spot;

        float shad2  = PointShadowFactor(ShadowCube2, light2_pos.xyz, p, n, lamb2);

        float up_amount = 0.5 + 0.5 * n.y;
        ambient  = Kd * mix(ambient_ground, ambient_sky, up_amount);

        diffuse  = Kd * lamb1 * att1 * light1_color * (1.0 - shad1)
                 + Kd * lamb2 * att2 * light2_color * (1.0 - shad2);

        specular = vec3(0.0);
        if (lamb1 > 0.0) specular += vec3(ks) * blinn1 * att1 * light1_color * (1.0 - shad1);
        if (lamb2 > 0.0) specular += vec3(ks) * blinn2 * att2 * light2_color * (1.0 - shad2);
    }

    color.rgb = ambient + diffuse + specular;
    color.a   = 1.0;

    // Correcao gamma, considerando monitor sRGB.
    color.rgb = pow(color.rgb, vec3(1.0)/2.2);
}
