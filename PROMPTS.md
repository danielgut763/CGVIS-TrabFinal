# Prompts de IA

Registro dos prompts usados para gerar código neste trabalho, conforme exigido
pelo enunciado da disciplina.

## Convenção adotada

- Todo commit cujo código foi gerado ou editado por IA tem a mensagem iniciada
  por `IA:`.
- O prompt correspondente é registrado neste arquivo, em uma seção nova, e
  commitado junto com o código que ele gerou.
- Código escrito por nós não aparece aqui e seus commits não levam o prefixo.
- `SPEC.md` e `README.md` não foram escritos com IA.

---

## Registro

<!--
Formato de cada entrada:

### <data> — <arquivo(s) afetado(s)>

**Commit:** `<hash curto>`

**Prompt:**

```
<prompt literal enviado à ferramenta>
```

**Ferramenta:** <nome>

**O que foi aproveitado:** <ajustes manuais feitos em cima da saída>
-->

### 2026-10-24 — M0: modularização, cena por instâncias, sala e delta time

**Commit:** `6ee5980`

**Arquivos gerados/alterados por IA:** `src/scene.h`, `src/scene.cpp`,
`src/collisions.h`, `src/collisions.cpp`, `data/cube.obj`,
`src/shader_fragment.glsl`, `src/main.cpp`, `include/matrices.h`,
`CMakeLists.txt`

**Prompt:**

```
Marco M0 do plano de implementação: a partir do template do laboratório,
criar um módulo de cena com uma struct Instance (malha + posição + rotação +
escala + material) e uma lista global desenhada por um laço, no lugar das
chamadas fixas de DrawVirtualObject; montar a sala com paredes, piso e teto
como caixas escaladas a partir de um cubo unitário, com um vão de saída
elevado na parede +X; reescrever o fragment shader trocando o if/else de
object_id por material por instância, com UV derivada da posição em mundo
para que a textura não estique em paredes de tamanhos diferentes; criar
collisions.cpp com AABB, AABB x AABB e raio x AABB; e introduzir delta time
no laço principal.
```

**Ferramenta:** Claude (Anthropic)

**O que foi aproveitado:** <preencher após revisão — o que foi mantido, o que
foi corrigido à mão e por quê>

### 2026-10-28 — M1: câmera 1ª pessoa, movimentação, gravidade e colisão por eixo

**Commit:** `e25e579`

**Arquivos gerados/alterados por IA:** `src/camera.h`, `src/camera.cpp`,
`src/player.h`, `src/player.cpp`, `src/collisions.h`, `src/collisions.cpp`,
`src/main.cpp`, `CMakeLists.txt`

**Prompt:**

```
Implemente a M1-andar do plano da entrega parcial, contido no plano.md
```

**Ferramenta:** Gemini (Google Antigravity)

**O que foi aproveitado:** <preencher após revisão — o que foi mantido, o que
foi corrigido à mão e por quê>

### 2026-10-30 — M2: picking, carregar objeto e perspectiva forçada

**Commit:** `db1af73`

**Arquivos gerados/alterados por IA:** `src/grab.h`, `src/grab.cpp`,
`src/main.cpp`, `CMakeLists.txt`

**Prompt:**

```
Marco M2 do plano de implementação, sobre o M1 já existente: criar um módulo
grab com a mecânica de perspectiva forçada do Superliminal. Construir o raio
do cursor a partir da base ortonormal da câmera (sem gluUnProject nem
glm::unProject, que são proibidas); fazer picking por raio x AABB escolhendo
o objeto marcado como grabbable; enquanto carregado, manter o objeto no mesmo
ponto da tela com tamanho aparente constante, aplicando escala proporcional à
distância; ao posicionar, não usar o ponto atingido pelo raio diretamente, e
sim resolver a forma fechada d = t_hit / (1 + r0/d0) e depois refinar por
busca binária testando a AABB do objeto contra a cena; travar o FOV enquanto
carrega; permitir rotação do objeto na mão com Q e E e ajuste de alcance pela
roda do mouse.
```

**Ferramenta:** Claude (Anthropic)

**Validação:** a matemática foi conferida por um teste numérico fora do
repositório, verificando escala proporcional à distância, invariância do raio
angular, contato com a parede sem penetração e recuo diante de obstáculo
lateral.

**O que foi aproveitado:** <preencher após revisão>

### 2026-10-30 — M3: curva de Bézier cúbica, câmera look-at e fechamento da parcial

**Commit:** `<preencher com o hash>`

**Arquivos gerados/alterados por IA:** `src/bezier.h`, `src/bezier.cpp`,
`src/camera.h`, `src/camera.cpp`, `src/grab.h`, `src/grab.cpp`,
`src/scene.h`, `src/scene.cpp`, `src/main.cpp`, `CMakeLists.txt`

**Prompt:**

```
leia todos os arquivos para pegar o contexto do trabalho, e depois excute o passo M3-Fechar a parcial do plano
```

**Ferramenta:** Gemini (Google Antigravity)

**O que foi aproveitado:** <preencher após revisão — o que foi mantido, o que
foi corrigido à mão e por quê>

### 2026-10-31 — M4: shadow mapping (funcionalidade extra)

**Commit:** `<preencher com o hash>`

**Arquivos gerados/alterados por IA:** `src/shadow.h`, `src/shadow.cpp`,
`src/shader_shadow_vertex.glsl`, `src/shader_shadow_fragment.glsl`,
`src/shader_fragment.glsl`, `src/main.cpp`, `CMakeLists.txt`

**Prompt:**

```
Marco M4 do plano de implementação: implementar shadow mapping como a
funcionalidade extra obrigatória, que o professor pediu explicitamente.
Criar um módulo shadow com framebuffer e textura de profundidade, com as
matrizes View e Projection da luz construídas por Matrix_Camera_View e
Matrix_Perspective de matrices.h (sem funções prontas). Tratar a luz como um
holofote no teto apontando para baixo, usando a mesma posição já usada no
cálculo de iluminação para que sombra e sombreamento fiquem coerentes.
Adicionar um par de shaders que só grava profundidade do ponto de vista da
luz, e no fragment shader principal projetar o fragmento no espaço da luz,
comparar profundidades com bias dependente da inclinação contra shadow acne e
suavizar a borda com PCF 3x3. Renderizar o passo de profundidade depois de
toda a lógica do quadro e antes do passo de cor, e permitir ligar e desligar
a sombra por tecla.
```

**Ferramenta:** Claude (Anthropic)

**Validação:** os dois programas de GPU foram compilados e linkados fora do
jogo, via WebGL2 (GLSL ES 3.00, mais estrito que o 3.30 desktop), e a lista de
uniforms ativos foi cruzada com os nomes buscados em main.cpp.

**O que foi aproveitado:** <preencher após revisão>

### 2026-10-31 — Correção do M4 e revisão visual da fase

**Commit:** `<preencher com o hash>`

**Arquivos gerados/alterados por IA:** `src/shadow.h`, `src/shadow.cpp`,
`src/shader_fragment.glsl`, `src/scene.h`, `src/scene.cpp`, `src/bezier.cpp`,
`src/main.cpp`

**Prompt:**

```
A sombra está cortando nas paredes e a iluminação não deixa claro de onde a
luz vem. Corrigir e revisar o visual da fase: trocar o mapa de sombra de
projeção perspectiva (cone a partir de uma luz dentro da sala, que nunca
alcança o topo das paredes) para projeção ortográfica de uma luz direcional,
com o volume ajustado à caixa que envolve a fase; usar ambiente hemisférico
para as paredes não ficarem chapadas; criar uma luminária visível embutida no
teto com material emissivo, que não projeta sombra; substituir as texturas de
tijolo e terreno do cenário por texturas procedurais na paleta da imagem de
referência do SPEC (paredes creme em painéis com juntas e rodapé verde, piso
verde com grade, teto claro em painéis); e ampliar a sala para 12 x 9 x 4,2 m
reduzindo o tamanho dos objetos, reposicionando a curva de Bézier e o jogador.
```

**Ferramenta:** Claude (Anthropic)

**Validação:** os dois programas de GPU foram recompilados via WebGL2 e os
uniforms cruzados com main.cpp. Um teste numérico fora do repositório
confirmou que os 8 cantos da caixa da fase, o topo das quatro paredes e um
objeto ampliado junto ao canto caem todos dentro do cubo NDC da luz — ou seja,
não existe mais ponto fora do volume onde a sombra pudesse cortar.

**Correção durante a revisão:** com a luz passando a ser direcional e vinda
de cima, o teto da sala passou a bloquear a luz no passo de profundidade e
toda a cena era declarada em sombra. Foi acrescentado o campo `casts_shadow`
em `Instance`: a casca da sala e as luminárias recebem sombra mas não a
projetam, e apenas os objetos da fase entram no passo de profundidade. A
filtragem do mapa também passou de GL_LINEAR para GL_NEAREST, já que
interpolar profundidades antes da comparação erra nas silhuetas.

**Segunda correção:** as faces viradas para a luz apareciam escurecidas por
shadow acne, porque faces quase de perfil ocupam poucos texels no mapa e a
profundidade varia demais dentro de cada um. Foi adicionado *normal offset
bias*: o ponto é deslocado ao longo da normal antes de ser projetado no espaço
da luz, com deslocamento crescente conforme a luz fica rasante. As duas faixas
de luminária foram unificadas em uma só, porque duas fontes visíveis
contradiziam a direção única das sombras e o SPEC define uma luz principal.

**Terceira correção (bug de estado de OpenGL):** o coelho e a esfera
renderizavam pretos. Causa: `ShadowMapInit` criava a textura de profundidade
sem chamar `glActiveTexture` antes, agindo sobre a unidade de textura que
`LoadTextureImage` havia deixado ativa (a 1, do rocky_terrain); o
`glBindTexture(..., 0)` do final desligava aquela unidade, e todo objeto que
usava a imagem 1 passava a ler preto. Corrigido selecionando a unidade do mapa
de sombra antes de criar a textura e restaurando a unidade 0 depois.

**Quarta correção (modelo de luz):** uma luz direcional tem a mesma direção
em todo ponto da sala, então objetos situados exatamente sob a luminária
recebiam luz inclinada, o que é fisicamente impossível. A iluminação passou a
usar uma LUZ EM LINHA: o shader procura, para cada fragmento, o ponto mais
próximo do segmento que representa a faixa de luz, de modo que a direção varie
com a posição — vertical sob a faixa, inclinada ao lado dela. Foi acrescentada
atenuação linear fraca com a distância até o segmento. O mapa de sombra
continua ortográfico, com direção própria quase vertical, porque uma projeção
ortográfica exige uma direção única.

**Ajuste de cena pedido:** a faixa central foi reduzida a 10% do tamanho e
foi acrescentada uma segunda luminária, um pendente em cone na quina oposta à
porta. O cone é uma malha nova (`data/cone.obj`) gerada por revolução com 32
segmentos. Como a faixa curta passou a se comportar como fonte pontual, a sua
atenuação ganhou termo quadrático, e as intensidades foram recalibradas
medindo o render para não saturar. A segunda luz ilumina mas não projeta
sombra: isso exigiria um segundo mapa de profundidade e um passo de
renderização a mais por quadro.

**Quinta correção (sombra onidirecional):** a sombra não acompanhava a luz.
A iluminação usava a posição da luminária, mas o mapa de sombra usava uma
direção fixa quase vertical, então a sombra caía sempre reto para baixo
independentemente de onde o objeto estivesse em relação à lâmpada. Foi
substituído por SHADOW CUBE MAP, um por luminária: seis renderizações de 90
graus gravando a distância euclidiana até a primeira superfície em cada
direção. A consulta no shader é feita por direção, então a sombra sai
radialmente da fonte. As duas luzes passaram a ser pontuais, com a mesma
posição usada no sombreamento e no mapa. Um cone único apontado para baixo não
serviria: um objeto erguido perto do teto e longe do eixo fica a quase 90
graus da lâmpada e cairia fora de qualquer cone possível.

**Spotlight (a partir do material da disciplina):** a Aula 17-18, páginas 79
a 88, define fonte pontual, spotlight e direcional, e dá o teste do cone — um
ponto não é iluminado quando dot((p−l)/|p−l|, v) < cos(alpha). O pendente, que
é uma cúpula em cone, passou de fonte pontual a spotlight apontando para
baixo, com abertura de 45 graus e borda suavizada por smoothstep em vez do
corte seco.

**Rebalanceamento do ambiente:** com uma caixa grande tapando a luminária
central, a sala quase não escurecia. A sombra estava correta; o que sustentava
a cena era o termo ambiente, que representa luz já refletida e por isso não é
cortado por sombra. Medindo quatro níveis no render, 0.30/0.17 com luzes em
1.70 ficou como melhor ponto: bloquear a luz central passou a escurecer a sala
em 29%, contra 22% antes, sem saturar a imagem. Também foram colocados tetos
no normal offset, no bias e no raio do PCF, para que superfícies muito
rasantes não descolem a sombra do objeto e para que sombras distantes não
virem borrão.

**Dois defeitos no passo de profundidade:** (1) o passo herdava o backface
culling do desenho normal, então um objeto que ENVOLVE a luz não projetava
sombra alguma — de dentro dele só se enxergam as faces de trás, que eram
descartadas. Medido: caixa envolvendo a luminária escurecia a sala só 1,8%;
com o culling desligado no passo de profundidade, 35,4%. (2) O near plane do
cube map era 0,05 m, e qualquer parte de um objeto encostado na luminária era
recortada e deixava de projetar; reduzido para 0,02 m, o que custa pouca
precisão porque o mapa guarda distância em textura float e o z-buffer só
ordena os objetos entre si.

**O que foi aproveitado:** <preencher após revisão>


### 2026-11-08 — M5: assets (peão por revolução, dado, cubo mágico, texturas), instâncias e Gouraud vs Blinn-Phong

**Commit:** `<preencher com o hash>`

**Arquivos gerados/alterados por IA:** `data/pawn.obj`, `data/die.obj`,
`data/rubik.obj`, `data/die_diff.jpg`, `data/rubik_diff.jpg`,
`data/pawn_wood_diff.jpg`, `data/FONTES.txt`, `src/scene.h`, `src/scene.cpp`,
`src/shader_vertex.glsl`, `src/shader_fragment.glsl`, `src/main.cpp`

**Prompt:**

```
execute the plan phase M5
```

**Ferramenta:** Gemini (Google Antigravity)

**O que foi aproveitado:** <preencher após revisão — o que foi mantido, o que
foi corrigido à mão e por quê>


### 2026-11-09 — M6: a fase (botão, porta, saída) e gravidade nos objetos soltos

**Commit:** `<preencher com o hash>`

**Arquivos gerados/alterados por IA:** `src/physics.h`, `src/physics.cpp`
(novos), `src/level.h`, `src/level.cpp` (novos), `src/collisions.h`,
`src/collisions.cpp`, `src/scene.h`, `src/scene.cpp`, `src/grab.h`,
`src/grab.cpp`, `src/player.h`, `src/main.cpp`, `src/shader_fragment.glsl`,
`CMakeLists.txt`

**Prompt:**

```
fiz o pull do m5
analisa e vamos para o m6 - adiciona no m6 a gravidade. Quando soltar o click, o objeto cai
```

**Ferramenta:** Claude (Anthropic)

**Resumo do que foi gerado:**

- Gravidade nos objetos pegáveis (`physics.cpp`): Euler semi-implícito, igual
  ao jogador. O apoio é procurado por `FindSupportHeight` em `collisions.cpp`
  (piso, topo de outro objeto, plataforma, cabeça do jogador), com tolerância
  de 2 cm para que um objeto pousado com folga de milímetros não atravesse o
  piso.
- Controle de segurar para carregar (`GrabConfig::HOLD_TO_CARRY`): apertar o
  botão esquerdo pega, soltar larga e o objeto cai. Se não couber onde está,
  a largada fica pendente e é tentada a cada quadro.
- Botão no piso (não sólido, afunda quando algo está em cima), com três
  estados: vermelho, âmbar (algo leve demais) e verde (objeto com altura
  >= 1 m). A porta desce para dentro da parede enquanto o botão está ativo e
  só volta a subir se não for prender o jogador ou um objeto.
- Corredor do lado de fora da saída, com painel luminoso no fundo; chegar
  nele conclui a fase. Tecla N reinicia.
- `JUMP_SPEED` de 4.2 para 4.8 m/s (pulo de ~0,73 m para ~0,96 m), para a
  saída a 2,2 m ser alcançável com dois objetos empilhados.
- Texturas procedurais da porta e do botão em coordenadas do modelo, porque
  a porta se move.

**Validação:** teste sem janela (lógica pura, com as mesmas funções do jogo):
dado solto a 2 m pousa em 0,550 s (teórico 0,552 s) exatamente no piso; dado
pequeno no botão só deixa âmbar; dado de 1,5 m abre a porta; jogador sobe
degrau → bloco → saída e vence; objeto que cai para fora volta ao lugar.
Shaders compilados e linkados em WebGL2.

**O que foi aproveitado:** <preencher após revisão>
