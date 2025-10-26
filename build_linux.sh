#!/bin/sh

set -xe

mkdir -p build

CFLAGS="-Wall -Wextra -ggdb -fsanitize=address -I./Dependencies/raylib-5.5_linux_amd64/include"
LFLAGS="-L./Dependencies/raylib-5.5_linux_amd64/lib -l:libraylib.a -lm"

gcc $CFLAGS -o build/pid Source/PID/pid_main.c $LFLAGS
