#!/bin/bash
# Checagem comum a todos: compila os modulos sem OpenGL + um teste, e faz
# -fsyntax-only do main.cpp. Uso: tools/check.sh [test/arquivo.cpp]
# Roda a partir da raiz da worktree.
#
# CHECK_DEFS="-DPHYSICS_ENABLE_TIPPING=0 -DPHYSICS_ENABLE_ROLLING=0" compila
# com as flags do M8 desligadas (comportamento do M7).
# test/t_m8.cpp roda duas vezes: flags ligadas (cenarios a-f, h-j) e
# desligadas (cenario g, comparacao com o M7).
set -e
cd "$(dirname "$0")/.."
T=${1:-test/t_base.cpp}
DEFS=${CHECK_DEFS:-}
MODS="src/collisions.cpp src/scene.cpp src/physics.cpp src/level.cpp src/player.cpp src/grab.cpp src/camera.cpp src/bezier.cpp"
for f in src/rigid.cpp src/tip.cpp src/roll.cpp; do [ -f $f ] && MODS="$MODS $f"; done
mkdir -p build_test
CXX="g++ -std=c++11 -O1 -g -Wall -Wextra -Wno-unused-parameter -Iinclude -Isrc"
$CXX $DEFS $MODS test/stub.cpp $T -o build_test/t
g++ -std=c++11 -fsyntax-only -Wall -Wno-unused-function -Wno-deprecated-declarations -Iinclude -Isrc src/main.cpp
echo "== compilou; rodando $T $DEFS"
rc=0
./build_test/t || rc=$?
if [ "$T" = "test/t_m8.cpp" ] && [ -z "$DEFS" ]; then
    OFF="-DPHYSICS_ENABLE_TIPPING=0 -DPHYSICS_ENABLE_ROLLING=0"
    $CXX $OFF $MODS test/stub.cpp $T -o build_test/t_off
    echo "== rodando $T com $OFF"
    ./build_test/t_off || rc=$?
fi
exit $rc
