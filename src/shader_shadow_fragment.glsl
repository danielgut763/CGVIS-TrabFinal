#version 330 core

// Grava a DISTANCIA da luz ate a superficie, normalizada pelo alcance.
//
// Gravar distancia euclidiana, em vez do z do buffer de profundidade, torna a
// comparacao no shader principal trivial: ele mede a distancia do fragmento
// ate a luz e compara com este valor, sem precisar refazer projecao nenhuma.

in vec3 world_position;

uniform vec3  light_position;
uniform float far_range;

out float frag_distance;

void main()
{
    frag_distance = length(world_position - light_position) / far_range;
}
