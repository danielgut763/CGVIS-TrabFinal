#ifndef LEVEL_H
#define LEVEL_H

// Logica da fase (M6): botao, porta da saida elevada e condicao de vitoria.
//
// Fluxo do puzzle, como descrito nas limitacoes do SPEC.md:
//   1. Um objeto pegavel grande o suficiente (altura >= MIN_ACTIVATE_HEIGHT)
//      apoiado sobre o botao o deixa verde e abre a porta.
//   2. A saida fica a 2,2 m do chao, fora do alcance do pulo. O mesmo objeto
//      grande, mais um menor servindo de degrau, viram a escada.
//   3. Chegar ao fundo do corredor do lado de fora conclui a fase.
//
// Os testes de "esta em cima do botao" e de "a porta pode fechar" sao
// sobreposicao de AABBs (collisions.cpp).

#include <vector>

#include "collisions.h"
#include "scene.h"

namespace LevelConfig
{
    const float KILL_Y = -5.0f; // abaixo disto um objeto ou o jogador voltou ao inicio
}

struct LevelState
{
    std::vector<Instance> initial; // copia da cena inicial, para reiniciar e
                                   // devolver objetos que cairam para fora

    glm::vec3 door_closed_pos;     // posicao da porta fechada
    float door_offset;             // quanto a porta ja desceu (0 = fechada)

    float button_press;            // animacao do botao, 0 = solto, 1 = no fundo
    bool  button_pressed;          // ha algo sobre o botao
    bool  button_active;           // e esse algo e grande o bastante: porta abre

    bool  won;
    float elapsed;                 // tempo de jogo, em segundos
    float win_time;                // tempo em que a fase foi concluida
};

// Guarda o estado inicial da cena. Chamar logo depois de BuildScene.
void LevelInit(LevelState& L);

// Volta a cena ao estado inicial (tecla N).
void LevelRestart(LevelState& L);

// Avanca a logica da fase por um passo "dt".
//   held_instance: objeto na mao do jogador (-1 se nenhum); nao conta como
//                  apoiado no botao enquanto esta sendo carregado.
void LevelUpdate(LevelState& L, int held_instance, const AABB& player_box, float dt);

// Verdadeiro se o jogador esta no corredor da saida.
bool LevelPlayerAtExit(const AABB& player_box);

#endif // LEVEL_H
