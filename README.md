# Infraliminal — Trabalho Final de Computação Gráfica e Visualização I (INF01047)

Reimplementação de uma fase de puzzle em primeira pessoa inspirada no jogo *Superliminal*, em que o tamanho real dos objetos em cena é alterado proporcionalmente à distância em que são posicionados a partir da câmera (perspectiva forçada), mantendo o tamanho aparente constante na tela.

---

## Integrantes da Dupla

- **Daniel Gutschwager** — Cartão UFRGS: `00315708`
- **Leonardo Leites** — Cartão UFRGS: `00338804`

### Contribuições da Dupla

- **Daniel Gutschwager**: Especificação inicial, câmera livre em primeira pessoa e câmera *look-at* (`src/camera.*`), movimentação e física do jogador com colisão resolvida por eixo (`src/player.*`, `src/collisions.*`), animação por curva de Bézier cúbica (`src/bezier.*`), modelagem/geração dos *assets* complexos (peão por revolução, dado, cubo mágico e texturas em atlas) e interpolação de Gouraud vs. Phong (`src/shader_vertex.glsl`, `src/shader_fragment.glsl`).
- **Leonardo Leites**: Especificação inicial, arquitetura de cena baseada em instâncias e texturas procedurais (`src/scene.*`), mecânica de *picking* por raio e perspectiva forçada com recuo de volume (`src/grab.*`), *shadow mapping* onidirecional por *cube map* com PCF (`src/shadow.*`, `src/shader_shadow_*.glsl`), gravidade dos objetos soltos (`src/physics.*`), lógica do puzzle da fase — botão, porta deslizante e condição de vitória (`src/level.*`) — e acabamento geral (`src/main.cpp`).

> O registro completo do uso de ferramentas de IA generativa e dos respectivos prompts encontra-se em [`PROMPTS.md`](PROMPTS.md). As fontes e licenças dos modelos e texturas estão documentadas em [`data/FONTES.txt`](data/FONTES.txt).

---

## Como Jogar e Resolver a Fase

O objetivo da fase é alcançar o corredor de saída elevado (a $2{,}2\text{ m}$ de altura na parede $+X$), cuja porta metálica começa fechada:

1. **Abrir a porta da saída**: aponte a mira central (`+`) para um dos objetos pegáveis da sala (dados, cubos mágicos, peões, bloco de tijolos ou coelho), segure o **botão esquerdo do mouse** para carregá-lo e mire em uma superfície distante (ou use a **roda do mouse**) para ampliar sua escala real mantendo o tamanho aparente na tela. Solte o objeto sobre o **botão no piso** com altura $\ge 1{,}0\text{ m}$: o botão mudará de vermelho (livre) ou âmbar (objeto leve demais) para **verde**, deslizando a porta metálica para baixo.
2. **Construir uma escada**: como o vão da saída está a $2{,}2\text{ m}$ do chão (acima da altura máxima de um pulo isolado, $\approx 0{,}96\text{ m}$), utilize o próprio objeto grande posicionado sobre o botão e posicione um segundo objeto de tamanho intermediário ($\approx 0{,}7\text{ m}$ a $0{,}9\text{ m}$) ao lado dele como primeiro degrau.
3. **Alcançar a saída**: suba no primeiro objeto (`Espaço` + `WASD`), salte para o topo do objeto maior sobre o botão e, a partir dele, pule para dentro do corredor elevado da saída até tocar o painel luminoso ao fundo.

---

## Controles (Teclado e Mouse)

| Entrada | Ação |
|---|---|
| **Mouse** | Olhar ao redor (câmera em 1ª pessoa) ou orbitar o objeto observado (câmera *look-at*) |
| **Botão esquerdo do mouse (segurar / soltar)** | Pegar o objeto sob a mira (*picking*) e soltá-lo com a nova escala na posição calculada |
| **Roda do mouse (*scroll*)** | Ajustar o alcance do objeto carregado (em 1ª pessoa com objeto na mão), dar *zoom* (em 1ª pessoa livre) ou ajustar o raio da órbita (na câmera *look-at*) |
| **`W` `A` `S` `D`** | Caminhar pelo cenário (1ª pessoa) ou orbitar continuamente o alvo pelo teclado (*look-at*) |
| **`Espaço`** | Pular (permitido quando apoiado no piso ou sobre um objeto sólido) |
| **`Q` / `E`** | Rotacionar o objeto carregado em torno do eixo $Y$ |
| **`C`** | Alternar entre a **câmera em 1ª pessoa** e a **câmera *look-at*** (foca no objeto carregado ou no objeto da curva de Bézier) |
| **`V`** | Trocar o objeto observado pela câmera *look-at* |
| **`L`** | Ligar / desligar as sombras (*shadow mapping*) em tempo real |
| **`P` / `O`** | Alternar entre projeção **perspectiva** e projeção **ortográfica** |
| **`N`** | Reiniciar a fase (restaura objetos, porta, botão e jogador ao estado inicial) |
| **`H`** | Mostrar / ocultar o HUD e textos informativos na tela |
| **`R`** | Recarregar e recompilar os shaders GLSL em tempo de execução |
| **`TAB`** | Capturar / liberar o cursor do mouse na janela |
| **`ESC`** | Fechar a aplicação |

---

## Arquitetura do Projeto

```text
src/
  main.cpp                     Loop principal, inicialização OpenGL, callbacks e HUD
  scene.cpp / scene.h          Estrutura Instance, materiais, geometria da sala e objetos
  camera.cpp / camera.h        Câmera em 1ª pessoa (yaw/pitch) e câmera look-at (órbita esférica)
  player.cpp / player.h        Física do jogador (WASD, pulo, gravidade Euler semi-implícito)
  grab.cpp / grab.h            Picking por raio, perspectiva forçada e recuo de colisão
  collisions.cpp / collisions.h Testes de intersecção (Raio×AABB, AABB×AABB, Ponto×AABB, suporte)
  physics.cpp / physics.h      Gravidade e empilhamento dos objetos soltos
  level.cpp / level.h          Lógica do puzzle (botão de 3 estados, porta deslizante e vitória)
  bezier.cpp / bezier.h        Curva de Bézier cúbica para o objeto decorativo flutuante
  shadow.cpp / shadow.h        Shadow cube maps onidirecionais (FBO, 6 faces de 90° por luz)
  shader_vertex.glsl           Vertex shader principal (transformações e iluminação de Gouraud)
  shader_fragment.glsl         Fragment shader principal (Blinn-Phong, texturas procedurais e PCF)
  shader_shadow_vertex.glsl    Vertex shader do passo de distância do shadow cube map
  shader_shadow_fragment.glsl  Fragment shader que grava distância euclidiana normalizada até a luz
```

---

## Atendimento aos Requisitos Técnicos

1. **Malhas poligonais complexas (`data/`, `src/scene.cpp`)**: cena composta pelo coelho de Stanford (`bunny.obj`), peão de xadrez Staunton modelado por superfície de revolução com 48 segmentos e 5664 triângulos (`pawn.obj`), cúpula pendente por revolução (`cone.obj`), dado chanfrado (`die.obj`), cubo mágico $3\times 3\times 3$ com sulcos entre peças (`rubik.obj`), esfera (`sphere.obj`) e caixas estruturais (`cube.obj`).
2. **Transformações geométricas controladas pelo usuário (`src/grab.cpp`, `src/scene.cpp`)**: todas as matrizes `Model`, `View` e `Projection` utilizam exclusivamente `include/matrices.h`. O jogador translada objetos ao carregá-los, rotaciona-os com `Q`/`E` (`Matrix_Rotate_Y`) e altera sua escala uniforme `Matrix_Scale` proporcionalmente à distância de pouso:
   $$s(d) = s_0 \cdot \frac{d}{d_0}$$
3. **Diferentes tipos de câmeras (`src/camera.cpp`)**: câmera livre em 1ª pessoa presa aos olhos do jogador e câmera *look-at* em coordenadas esféricas orbitando o objeto carregado ou qualquer objeto da cena (alternadas com `C` e `V`), com recuo automático contra as paredes da sala.
4. **Instâncias de objetos (`src/scene.cpp`)**: cada malha é carregada uma única vez na GPU (`g_VirtualScene`) e desenhada em múltiplas entradas de `g_Instances` com matrizes `Model` e materiais distintos (3 dados, 2 cubos mágicos, 2 peões de xadrez e dezenas de caixas estruturais).
5. **Testes de intersecção (`src/collisions.cpp`)**:
   - **Raio $\times$ AABB** (método das fatias / *slabs*): usado no *picking* de objetos sob a mira e no cálculo da distância de pouso do objeto carregado.
   - **AABB $\times$ AABB**: resolução de colisão eixo a eixo do jogador contra paredes, porta e objetos (`ResolveAABBMovement`), ativação do botão no piso (`level.cpp`), bloqueio de fechamento da porta sobre objetos/jogador e refinamento por busca binária do pouso na mecânica de escala (`grab.cpp`).
   - **Ponto $\times$ AABB** (`FindBoxContainingPoint`): detecção da caixa sólida que apoia os pés do jogador.
6. **Modelos de iluminação e interpolação (`src/shader_vertex.glsl`, `src/shader_fragment.glsl`)**: modelo de **Blinn-Phong** com ambiente hemisférico e duas fontes de luz coerentes com a geometria das luminárias (fonte pontual central no teto e *spotlight* direcionado para baixo com borda suavizada por `smoothstep` na cúpula pendente). A cena combina interpolação de **Phong** (por fragmento, na maioria dos objetos e no primeiro peão) e interpolação de **Gouraud** (avaliada por vértice no *vertex shader*, aplicada ao segundo peão de xadrez e à esfera da curva de Bézier para comparação direta lado a lado).
7. **Mapeamento de texturas (`src/main.cpp`, `src/shader_fragment.glsl`)**: todos os objetos utilizam mapeamento de texturas a partir de 5 imagens carregadas (`red_brick_diff_1k.jpg`, `rocky_terrain_02_diff_1k.jpg`, `die_diff.jpg`, `rubik_diff.jpg`, `pawn_wood_diff.jpg`) combinadas com texturas procedurais em coordenadas de mundo (`UV_WORLD`, mantendo texels quadrados sem esticamento em paredes e pisos de qualquer dimensão), coordenadas UV de atlas (`UV_OBJ`), projeção esférica (`UV_SPHERE`) e projeção tri-planar no espaço do modelo (`UV_PLANAR_XY`).
8. **Curva de Bézier cúbica (`src/bezier.cpp`)**: esfera decorativa flutuante percorrendo continuamente (ida e volta) uma trajetória tridimensional em "S" definida por polinômios de Bernstein de grau 3 na parte superior da sala.
9. **Animações baseadas no tempo — $\Delta t$ (`src/main.cpp`)**: toda movimentação da aplicação (caminhada, gravidade do jogador, queda dos objetos soltos, rotação do objeto na mão, órbita da câmera *look-at*, curva de Bézier, animação do botão e deslocamento da porta) é integrada em função de `g_DeltaTime`.

---

## Funcionalidade Extra Obrigatória: Shadow Mapping Onidirecional e Picking com Perspectiva Forçada

Conforme especificado no `SPEC.md`, foram implementadas duas funcionalidades extras de Computação Gráfica:

### 1. Shadow Mapping Onidirecional (*Point Shadow Cube Maps* + PCF)
Implementado em `src/shadow.cpp`, `src/shader_shadow_vertex.glsl`, `src/shader_shadow_fragment.glsl` e `src/shader_fragment.glsl`:
- Cada uma das duas luminárias possui um *framebuffer* (`FBO`) acoplado a uma textura cúbica de ponto flutuante (`GL_TEXTURE_CUBE_MAP`, formato `GL_R32F`, $512 \times 512$ por face).
- A cada quadro, após toda a lógica de simulação, a cena é renderizada nas 6 direções ($\pm X, \pm Y, \pm Z$, abertura de $90^\circ$) de cada luminária utilizando `Matrix_Camera_View` e `Matrix_Perspective` de `include/matrices.h`.
- O *fragment shader* do passo de sombra grava a distância euclidiana normalizada $\|\mathbf{p}_{\text{world}} - \mathbf{l}\| / d_{\text{far}}$.
- No *fragment shader* principal, a sombra é avaliada radialmente em todas as direções com **Normal Offset Bias** e **Slope-Scaled Depth Bias** (eliminando *shadow acne* em paredes rasantes) e suavizada por filtragem **PCF de 9 amostras** tridimensionais ao redor do vetor direção da luz. Pode ser alternada ao vivo com a tecla **`L`**.

### 2. Ray Picking e Pouso com Volume na Perspectiva Forçada
Implementado em `src/grab.cpp`:
- O raio do cursor é construído diretamente a partir da base ortonormal da câmera ($\mathbf{u}, \mathbf{v}, \mathbf{w}$), sem utilizar `gluUnProject` ou `glm::unProject`.
- Enquanto o objeto é carregado, o campo de visão (`fov`) é travado e a direção local na base da câmera é preservada para que o objeto permaneça exatamente sob o mesmo ponto da tela.
- Para evitar que o objeto penetre na parede ou no piso ao crescer com a distância $d$, calcula-se primeiro a distância tangente em forma fechada considerando a meia-extensão efetiva $r_{\text{eff}}$ do objeto ao longo do raio e da normal da superfície atingida:
  $$d_{\text{wall}} = \frac{t_{\text{hit}}}{1 + r_{\text{eff}} / d_0}$$
  Em seguida, caso haja obstáculos laterais próximos, uma busca binária de 14 iterações recua $d$ até que a AABB escalada do objeto fique livre de qualquer intersecção com a cena e com o jogador.

---

## Compilação e Execução

As instruções detalhadas para cada sistema operacional também estão disponíveis em [`COMPILACAO.md`](COMPILACAO.md).

### Linux (CMake ou Makefile)
```bash
cmake -B build -S .
cmake --build build
cmake --build build -- run
```
Ou diretamente com `make run`.

### Windows (CMake / MinGW / VSCode)
```powershell
cmake -B build -S .
cmake --build build
.\bin\Debug\main.exe
```
> **Nota**: o executável carrega shaders e modelos usando caminhos relativos (`../../src/` e `../../data/`), portanto deve ser executado a partir do diretório `bin/Debug/`, `bin/Release/` ou `bin/Linux/` (o que já é feito automaticamente pelo alvo `run` e pela configuração do VSCode em `.vscode/launch.json`).
