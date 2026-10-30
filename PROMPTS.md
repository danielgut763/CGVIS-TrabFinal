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

**Commit:** `<preencher com o hash>`

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

**Commit:** `<preencher com o hash>`

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
