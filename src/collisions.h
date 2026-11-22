#ifndef COLLISIONS_H
#define COLLISIONS_H

// Testes de intersecao entre objetos virtuais.
//
// O enunciado exige que estes testes fiquem em um arquivo separado chamado
// "collisions.cpp". Este modulo NAO conhece a cena nem OpenGL: ele recebe
// caixas e raios por parametro. Quem monta a lista de caixas e main.cpp.
//
// Testes implementados e seu proposito na aplicacao:
//   1. Raio x AABB  -> escolher o objeto sob o cursor (picking) e descobrir
//                      onde o objeto carregado vai pousar.
//   2. AABB x AABB  -> impedir que o jogador atravesse parede, piso e objetos,
//                      e impedir que o objeto seja solto dentro da geometria.
//   3. Ponto em AABB-> detectar o jogador em cima de um objeto e sobre o botao.

#include <vector>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

struct AABB
{
    glm::vec3 min;
    glm::vec3 max;
};

struct RayHit
{
    bool      hit;    // houve intersecao com t >= 0
    float     t;      // distancia ao longo do raio (direcao normalizada)
    int       index;  // indice da caixa atingida dentro do vetor consultado
    glm::vec3 normal; // normal da face atingida, em mundo
};

// AABB em MUNDO de um objeto cuja bounding box LOCAL e [local_min, local_max]
// e cuja Model matrix e "model". Transforma os 8 cantos e devolve a caixa
// envolvente, o que mantem o resultado correto mesmo com rotacao em Y.
AABB AABBTransform(const glm::vec3& local_min,
                   const glm::vec3& local_max,
                   const glm::mat4& model);

// Cresce (ou encolhe, com margem negativa) a caixa em todas as direcoes.
AABB AABBExpand(const AABB& box, float margin);

glm::vec3 AABBCenter(const AABB& box);
glm::vec3 AABBHalfSize(const AABB& box);

// Meia-extensao da caixa projetada sobre a direcao "dir" (que deve estar
// normalizada). E o "raio" do objeto naquela direcao, usado no calculo de
// onde o objeto pousa na mecanica de perspectiva forcada.
float AABBHalfExtentAlongDir(const AABB& box, const glm::vec3& dir);

// Teste de sobreposicao entre duas caixas alinhadas aos eixos.
bool AABBOverlap(const AABB& a, const AABB& b);

bool AABBContainsPoint(const AABB& box, const glm::vec3& p);

// Raio x AABB pelo metodo das fatias (slabs). "dir" deve estar normalizada.
// Devolve em t_near/t_far as distancias de entrada e saida. Se a origem esta
// dentro da caixa, t_near vale 0.
bool RayAABB(const glm::vec3& origin, const glm::vec3& dir, const AABB& box,
             float* t_near, float* t_far, glm::vec3* normal);

// Lanca o raio contra uma lista de caixas e devolve a mais proxima.
// "skip_index" permite ignorar uma caixa (tipicamente o proprio objeto que
// esta sendo carregado); passe -1 para nao ignorar nenhuma.
RayHit RaycastBoxes(const glm::vec3& origin, const glm::vec3& dir,
                    const std::vector<AABB>& boxes, int skip_index);

// Verdadeiro se "box" encosta em qualquer caixa da lista, ignorando
// skip_index. Usado para validar a posicao de pouso do objeto carregado.
bool AABBOverlapsAny(const AABB& box, const std::vector<AABB>& boxes,
                     int skip_index);

// Mesma coisa, ignorando DUAS caixas (M8): tipicamente a do proprio objeto e
// a do apoio sobre o qual ele esta tombando ou rolando, que ele toca de
// proposito. Passe -1 em qualquer uma para nao ignorar.
bool AABBOverlapsAnyExcept(const AABB& box, const std::vector<AABB>& boxes,
                           int skip_a, int skip_b);

// Devolve o indice da primeira caixa da lista que contem o ponto "p"
// (ignorando skip_index), ou -1 se nenhuma contiver. Usado para detectar o
// jogador em cima de um objeto e sobre o botao.
int FindBoxContainingPoint(const glm::vec3& p,
                           const std::vector<AABB>& boxes,
                           int skip_index);

// Queda vertical de um objeto solto (M6). Procura, entre as caixas que se
// sobrepoem a "box" no plano XZ, a superficie de apoio mais alta que esteja
// abaixo da base da caixa, ou no maximo "tolerance" acima dela (o objeto pode
// ter sido solto encostando de leve em algo). Devolve false se nao houver
// nenhuma: o objeto esta sobre o vazio e cai indefinidamente.
//
// Isto ja e uma VARREDURA vertical: a busca nao tem limite de distancia para
// baixo, entao quem cai "dy" (negativo) e testa  box.min.y + dy <= altura
// encontra qualquer topo no intervalo [box.min.y + dy, box.min.y], por mais
// fino que seja o obstaculo e por maior que seja dy (physics.cpp).
bool FindSupportHeight(const AABB& box,
                       const std::vector<AABB>& boxes,
                       int skip_index,
                       float tolerance,
                       float* out_height,
                       int*   out_index = 0);

// Area de apoio (M7): retangulo no plano XZ formado pela uniao das regioes em
// que a base de "box" encosta nas caixas cujo topo esta na altura
// "support_y" (com tolerancia). Devolve false se nenhuma caixa apoia.
//
// Um objeto so fica em equilibrio se a projecao do seu centro de massa cair
// dentro da area de apoio. Como a sala e feita de caixas alinhadas aos eixos,
// a uniao retangular das regioes de contato e uma boa aproximacao do fecho
// convexo dos pontos de apoio (um objeto apoiado sobre dois outros, com o
// centro no vao entre eles, continua em pe, como na vida real).
bool SupportFootprint(const AABB& box,
                      const std::vector<AABB>& boxes,
                      int skip_index,
                      float support_y,
                      float tolerance,
                      float* out_min_x, float* out_max_x,
                      float* out_min_z, float* out_max_z);

// Raio x esfera (M7). Resolve |o + t*d - c|^2 = r^2 para t, com "dir"
// normalizada. Devolve o menor t >= 0 em *out_t. Usado para apontar a camera
// look-at para a esfera da curva de Bezier, que nao e uma caixa e nao entra
// na lista de AABBs solidas.
bool RaySphere(const glm::vec3& origin, const glm::vec3& dir,
               const glm::vec3& center, float radius, float* out_t);

// Move uma AABB por "delta" resolvendo colisoes contra "boxes" UM EIXO POR
// VEZ. Em cada eixo, se o deslocamento faria a caixa penetrar algum obstaculo
// solido (ignorando skip_index), o movimento naquele eixo e truncado na face
// de contato e "out_hit_axis[eixo]" recebe true.
// Devolve o deslocamento efetivamente aplicado.
//
// Sem tunelamento (M8): em cada eixo o teste e uma VARREDURA do intervalo
// [inicio, inicio + delta] (collisions.cpp, ResolveSingleAxis), e nao um teste
// da posicao final. Um obstaculo entre o inicio e o fim do passo e encontrado
// qualquer que seja o tamanho do passo: a 20 m/s com dt = 0,1 s (2 m por
// passo) a caixa ainda para na face de uma parede de 0,2 m ou da porta.
// Uma caixa que JA comeca interpenetrando um obstaculo (por exemplo, depois
// de um giro de tombar/rolar) pode sair dele, mas nao se aprofundar.
// test/t_b1.cpp verifica os dois casos.
glm::vec3 ResolveAABBMovement(const AABB& box,
                              const glm::vec3& delta,
                              const std::vector<AABB>& boxes,
                              int skip_index,
                              bool out_hit_axis[3]);

#endif // COLLISIONS_H
