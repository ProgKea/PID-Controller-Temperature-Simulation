#!/bin/sh

set -xe

mkdir -p build

CFLAGS="-Wall -Wextra -ggdb -fsanitize=address -I./Source $(pkg-config --cflags raylib)"
LFLAGS="$(pkg-config --libs raylib)"

clang $CFLAGS -o build/pid Source/PID/pid_main.c $LFLAGS
