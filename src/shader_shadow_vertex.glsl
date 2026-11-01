#version 330 core

// Passo de profundidade do shadow cube map.
//
// Renderiza a cena de dentro da lampada, uma face de cada vez. Precisamos da
// posicao em MUNDO no fragment shader para calcular a distancia ate a luz.

layout (location = 0) in vec4 model_coefficients;

uniform mat4 model;
uniform mat4 light_face_matrix;  // Projection * View da face atual

out vec3 world_position;

void main()
{
    vec4 world = model * model_coefficients;
    world_position = world.xyz;
    gl_Position = light_face_matrix * world;
}
