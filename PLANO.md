# Plano de implementação — Trabalho Final CGVis

Documento de trabalho interno da dupla. Não é o `SPEC.md` nem o `README.md`.

---

## 1. Ponto de partida

O repositório hoje é o template do laboratório, sem alteração de código.

**O que já vem pronto e será reaproveitado:**

| Item | Onde | Situação |
|---|---|---|
| Matrizes Model/View/Projection próprias | `include/matrices.h` | Completo. `Matrix_Camera_View`, `Matrix_Perspective`, `Matrix_Orthographic`, rotações, produto vetorial/escalar |
| Carregador de OBJ | `main.cpp` + `tiny_obj_loader` | Completo |
| AABB por malha | `BuildTrianglesAndAddToVirtualScene` | Já calcula `bbox_min`/`bbox_max` e manda pro shader |
| Carregamento de textura | `LoadTextureImage` (stb_image) | Completo |
| Registro de malhas | `g_VirtualScene` (map) | Completo |
| Texto na tela | `textrendering.cpp` | Útil pra debug e HUD |

**O que o template NÃO tem e precisa ser escrito:**

- Câmera livre em primeira pessoa (hoje só existe look-at orbitando a origem)
- Qualquer noção de instância: o desenho é hardcoded, não há lista de objetos da cena
- Iluminação além de Lambert chapado (`color = Kd * (lambert + 0.01)`)
- Colisão de qualquer tipo
- Δt (as animações usam `glfwGetTime()` direto)
- Sombra, Bézier, picking, a mecânica do jogo

O `object_id` do fragment shader é um `if/else` com três constantes (`SPHERE`,
`BUNNY`, `PLANE`) e UV procedural. Isso precisa virar algo baseado em material
por objeto, senão não escala para a quantidade de objetos da fase.

---

## 2. Arquitetura alvo

Dividir o `main.cpp` (hoje 70 KB) em módulos com interface clara. Isso não é
preciosismo: é o que permite os dois trabalharem em paralelo sem conflito de
merge, e o `collisions.cpp` é exigência nominal do enunciado.

```
src/
  main.cpp          loop principal, setup GL, callbacks, orquestração
  scene.cpp/.h      lista de instâncias, carga de malhas e texturas, desenho
  camera.cpp/.h     câmera livre (1ª pessoa) e câmera look-at
  player.cpp/.h     movimentação, gravidade, subir em objeto
  grab.cpp/.h       a mecânica: picking, carregar, escala por distância, soltar
  collisions.cpp/.h OBRIGATÓRIO por nome. Todos os testes de intersecção
  bezier.cpp/.h     curva cúbica e o objeto que anda nela
  shadow.cpp/.h     shadow map (FBO, matrizes da luz, render de profundidade)
```

**Estrutura central da cena** — cada objeto desenhado é uma instância:

```cpp
struct Instance {
    const char* mesh;      // chave em g_VirtualScene
    glm::vec3   pos;
    float       yaw;       // rotação em Y
    float       scale;     // UNIFORME (ver seção 3)
    int         material;  // índice de textura + parâmetros de iluminação
    bool        grabbable;
    bool        solid;     // entra nos testes de colisão
};
std::vector<Instance> g_Instances;
```

Com isso, instância de objeto (requisito) sai de graça: a mesma `mesh` aparece
em várias entradas com `Model` diferente.

A `Model` de cada instância:
`Matrix_Translate(pos) * Matrix_Rotate_Y(yaw) * Matrix_Scale(scale,scale,scale)`

**AABB em mundo** de uma instância: pega `bbox_min`/`bbox_max` da malha,
multiplica pela escala, rotaciona pelo yaw (se o yaw for múltiplo de 90° isso é
trivial; se for livre, recalcular a AABB envolvente dos 8 cantos).

---

## 3. O núcleo técnico: a mecânica de perspectiva forçada

Esta é a parte que o professor comentou duas vezes. Vale implementar com
cuidado e deixar comentada no código.

### 3.1 Por que a escala é linear na distância

Sob projeção perspectiva, a altura projetada de um objeto de altura `h` a uma
distância `d` da câmera é aproximadamente

```
h_tela  =  (h / d) * (altura_viewport / 2) / tan(fov/2)
```

Para o objeto manter o mesmo tamanho **aparente** na tela enquanto a distância
muda, é preciso que `h / d` seja constante. Logo:

```
s(d) = s0 * (d / d0)
```

onde `s0` e `d0` são a escala e a distância no instante em que o objeto foi
pego. A escala é **uniforme** nos três eixos — isso evita ter que recalcular
normais pela transposta da inversa, e evita distorcer a textura.

Duas consequências que precisam virar regra no código:

- **O FOV não pode mudar enquanto o objeto está na mão.** Se houver zoom, ele
  deve ser bloqueado durante o carregamento, senão a invariante quebra.
- **A distância é medida até o mesmo ponto de ancoragem** usado no momento de
  pegar — o centro da AABB do objeto, não a origem do OBJ, que costuma estar
  nos pés da malha.

### 3.2 Onde o objeto pousa (o comentário do professor)

Posicionar o objeto direto no ponto atingido pelo raio enterra metade dele na
parede, porque o objeto tem volume. E o volume cresce junto com a distância, o
que realimenta o problema.

Seja `r0` o "raio" do objeto (meia-extensão da AABB ao longo da direção do
raio) quando ele está a `d0`. A uma distância `d`, esse raio vale
`r0 * (d / d0)`. Queremos que a face da frente do objeto encoste na superfície,
não passe dela:

```
d + r0 * (d/d0)  <=  t_hit
```

Resolvendo para `d`, sai uma forma fechada:

```
d  =  t_hit / (1 + r0/d0)
```

Isso é um *sphere cast* de uma esfera que cresce proporcionalmente à distância,
e resolve o caso da parede de frente sem nenhuma iteração.

**Depois** disso, refinar para os casos laterais (corredor estreito, objeto já
posicionado no caminho): com `d` candidato, montar a AABB do objeto escalado e
testar contra a cena. Se houver intersecção, busca binária entre `d_min` e `d`,
umas 10 iterações, até ficar livre. É barato e cobre o que a fórmula fechada
não cobre.

### 3.3 Limites

- `scale` presa em `[s_min, s_max]`. Sem isso o objeto pode virar poeira ou
  engolir a fase, e um objeto gigante e distante briga com o z-buffer.
- `near = 0.1`, `far` só o necessário para cobrir a sala. Planos de recorte mal
  escolhidos são a causa número um de z-fighting, que é bug listado nominalmente
  nos critérios.

### 3.4 Construção do raio sem funções proibidas

`gluUnProject` e `glm::unProject` estão vetados. O raio do cursor se monta
direto, sem inverter matriz nenhuma:

1. Converter a posição do mouse para NDC: `x_ndc`, `y_ndc` em `[-1,1]`.
2. Direção em coordenadas de câmera:
   `dir_cam = (x_ndc * aspect * tan(fov/2), y_ndc * tan(fov/2), -1)`
3. Levar para mundo combinando os vetores da base da câmera (`right`, `up`,
   `-forward`), que já são conhecidos porque fomos nós que construímos a View.

Origem do raio = posição da câmera. Normalizar `dir`.

---

## 4. Requisitos, um a um

| # | Requisito | Como será atendido | Arquivo |
|---|---|---|---|
| 1 | Tempo real | Cena pequena, malhas decimadas, shadow map único | — |
| 2 | Lógica não trivial | Puzzle: aumentar objeto, empilhar, subir, alcançar a saída elevada | `player`, `grab` |
| 3 | Mouse + teclado | WASD + espaço (teclado), mirar/pegar/soltar/girar (mouse) | `main`, `camera` |
| 4 | Model/View/Projection próprias | `matrices.h` — já pronto | `matrices.h` |
| 5 | Malhas complexas | Peão modelado por revolução, boneco baixado CC0, cubo e dado modelados | `data/` |
| 6 | Transformações pelo usuário | Translação (carregar), rotação (girar na mão), escala (soltar) | `grab` |
| 7 | Duas câmeras | 1ª pessoa livre + look-at orbitando o objeto carregado | `camera` |
| 8 | Instâncias | Mesma malha (dado/cubo) em várias `Instance` com Model distinta | `scene` |
| 9 | Intersecção | Raio×cena (pouso), jogador×parede, jogador em cima de objeto, objeto×botão | `collisions.cpp` |
| 10 | Iluminação não trivial | Blinn-Phong por fragmento; um objeto com Gouraud para contraste | shaders |
| 11 | Texturas | ≥3 imagens, todos os objetos; tiling nas paredes contra esticamento | shaders, `scene` |
| 12 | Bézier cúbica | Objeto decorativo flutuante percorrendo a curva, ida e volta | `bezier` |
| 13 | Δt | Todo movimento multiplicado pelo delta do quadro, câmera incluída | todos |
| 14 | Extra | Shadow mapping (+ picking, que o SPEC também promete) | `shadow`, `grab` |

### Nota sobre textura esticada

Desconto nominal nos critérios. Nas paredes e piso, a UV deve ser derivada do
tamanho em mundo (`u = x / tamanho_textura`), com `GL_REPEAT`, para o texel
ficar quadrado independente da dimensão da parede. Não usar UV de `[0,1]`
esticada sobre a parede inteira.

### Nota sobre iluminação

A luz principal será uma só, para o shadow map ser único. Usar Blinn-Phong
(`(N·H)^n`) em vez de Phong clássico: é mais barato e o brilho se comporta
melhor. Manter um objeto com Gouraud (cálculo no vertex shader) deixa visível
que as duas formas de interpolação foram implementadas.

---

## 5. Cronograma

Hoje: **24/10**. Parcial: **04/11** (11 dias). Apresentação: **07 ou 09/12**.
Entrega: **11/12 às 23h55**.

### Até a parcial (04/11) — prioridade máxima

| Marco | Prazo | Conteúdo |
|---|---|---|
| **M0 — Fundação** | 24–26/10 | Quebrar `main.cpp` nos módulos; `struct Instance` e lista da cena; sala (paredes, piso, teto) como caixas com medidas reais; textura com tiling; Δt no loop |
| **M1 — Andar** | 27–29/10 | Câmera 1ª pessoa (yaw/pitch no mouse, WASD); AABB do jogador; colisão jogador×parede resolvida por eixo em `collisions.cpp`; gravidade com Euler semi-implícito |
| **M2 — A mecânica** | 30/10–02/11 | Raio do cursor; picking (raio×AABB, método das fatias); carregar; escala pela distância; pouso com recuo de colisão (seção 3.2); rotação do objeto na mão |
| **M3 — Fechar a parcial** | 03/11 | Objeto na curva de Bézier; segunda câmera (look-at); limpar o que estiver travando |

Chegando aqui, a parcial mostra **9 dos 14 requisitos** funcionando. É uma
apresentação forte.

### Depois da parcial

| Marco | Prazo | Conteúdo |
|---|---|---|
| **M4 — Sombras** | 05–15/11 | FBO de profundidade; View/Projection da luz com `matrices.h`; comparação no fragment shader; bias contra acne; PCF 3×3; frustum da luz acompanhando o objeto escalado |
| **M5 — Assets e luz** | 16–23/11 | Peão por revolução no Blender; cubo e dado com as fotos como textura; boneco CC0; Blinn-Phong completo; um objeto em Gouraud; instâncias |
| **M6 — A fase** | 24/11–01/12 | Desenho do puzzle: empilhar, subir, botão, saída elevada, condição de vitória; ajuste de dificuldade |
| **M7 — Acabamento** | 02–07/12 | Caça a bug: z-fighting, textura esticada, colisão travando, flickering; `README.md`; vídeo de 3 a 5 min; `metadados-entrega.json` |

Folga real: cerca de 4 dias. Realista, mas não sobra para refazer nada grande.

---

## 6. Divisão da dupla

Os dois precisam ter commits no histórico — é critério individual. A divisão
por módulo com interface definida evita conflito:

- **Trilha A:** `camera`, `player`, `collisions` — movimentação e física
- **Trilha B:** `scene`, `grab`, `shadow` — renderização e a mecânica
- **Juntos:** `main.cpp`, desenho da fase, caça a bug, vídeo

`collisions.cpp` é usado pelas duas trilhas; definir as assinaturas logo no M0
e não mexer depois.

---

## 7. Armadilhas conhecidas

- **`glm::rotate`, `glm::translate`, `glm::scale`, `glm::perspective`,
  `glm::lookAt`, `glm::ortho`** estão no projeto via `include/glm/gtc/` e são
  critério **eliminatório**. Usar exclusivamente `matrices.h`.
- **Commits de IA** precisam de prefixo `IA:` e do prompt em `PROMPTS.md`.
  Combinar isso antes do primeiro commit de código, não depois.
- **Histórico incremental**: commit único no fim não é aceito.
- **Licença de asset**: todo modelo e textura baixados entram com fonte e
  licença registradas em `data/FONTES.txt`, que já existe.
- **Coerência com o `SPEC.md`**: o SPEC está congelado. Se a implementação
  divergir, a nota cai em dois critérios. Reler o SPEC antes do M6.
- **Normais de malha baixada**: se vierem erradas ou ausentes, o template já
  tem `ComputeNormals`.
