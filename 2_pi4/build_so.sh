#!/usr/bin/env bash
set -e

# Compile C++ Native engines on Raspberry Pi 4 ARM64
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -d "$SCRIPT_DIR/cpp" ]; then
    CPP_DIR="$SCRIPT_DIR/cpp"
else
    REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
    CPP_DIR="$REPO_ROOT/5_webots/simulation/my_project/controllers/tof_path_planner/cpp"
fi

cd "$SCRIPT_DIR"

echo "=== [1/2] Compiling smart_hybrid_costmap_front.so ==="
g++ -O3 -shared -fPIC -std=c++17 -mcpu=cortex-a72 -Wall -Wextra \
    -o smart_hybrid_costmap_front.so "$CPP_DIR/smart_hybrid_costmap_front.cpp"

echo "=== [2/2] Compiling OSQP & curvilinear_sl_mpc.so ==="
cd "$CPP_DIR/osqp_codegen_sl_mpc"
gcc -O3 -fPIC -mcpu=cortex-a72 -I. -Iinc/public -Iinc/private -c workspace.c src/*.c emosqp.c
ar rcs libosqp.a *.o src/*.o
cd "$SCRIPT_DIR"

g++ -O3 -shared -fPIC -std=c++17 -mcpu=cortex-a72 -fno-math-errno \
    -I"$CPP_DIR" -I"$CPP_DIR/osqp_codegen_sl_mpc" -I"$CPP_DIR/osqp_codegen_sl_mpc/inc/public" -I"$CPP_DIR/osqp_codegen_sl_mpc/inc/private" \
    -o curvilinear_sl_mpc.so "$CPP_DIR/curvilinear_sl_mpc.cpp" "$CPP_DIR/osqp_codegen_sl_mpc/libosqp.a" -lm

echo "=== All shared libraries compiled successfully ==="
if [ -d "$REPO_ROOT/brain" ]; then
    cp *.so "$REPO_ROOT/brain/"
    echo "=== Copied .so libraries to brain/ ==="
fi
ls -lh *.so
