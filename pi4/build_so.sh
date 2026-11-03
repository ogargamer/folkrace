#!/usr/bin/env bash
set -e

# Compile C++ Native engines on Raspberry Pi 4 ARM64
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== [1/2] Compiling smart_hybrid_costmap_front.so ==="
g++ -O3 -shared -fPIC -std=c++17 -mcpu=cortex-a72 -Wall -Wextra \
    -o smart_hybrid_costmap_front.so cpp/smart_hybrid_costmap_front.cpp

echo "=== [2/2] Compiling OSQP & curvilinear_sl_mpc.so ==="
cd cpp/osqp_codegen_sl_mpc
gcc -O3 -fPIC -mcpu=cortex-a72 -I. -Iinc/public -Iinc/private -c workspace.c src/*.c emosqp.c
ar rcs libosqp.a *.o src/*.o
cd ../..

g++ -O3 -shared -fPIC -std=c++17 -mcpu=cortex-a72 -fno-math-errno \
    -Icpp -Icpp/osqp_codegen_sl_mpc -Icpp/osqp_codegen_sl_mpc/inc/public -Icpp/osqp_codegen_sl_mpc/inc/private \
    -o curvilinear_sl_mpc.so cpp/curvilinear_sl_mpc.cpp cpp/osqp_codegen_sl_mpc/libosqp.a -lm

echo "=== All shared libraries compiled successfully ==="
ls -lh *.so

