#! /bin/bash

set echo on
output="app"

debug=1
simd_256=0

SRC_DIR=$PWD

mkdir -p build
cd build

#c_files=$(find $SRC_DIR/ -type f -name "*.c")
c_files="$SRC_DIR/app/main.c $SRC_DIR/app/app.c $SRC_DIR/app/camera.c $SRC_DIR/app/default_pass.c $SRC_DIR/app/gravity.c $SRC_DIR/lib/glad/src/glad.c $SRC_DIR/lib/mt19937/src/mt19937.c $SRC_DIR/lib/mt19937/src/mt19937_64.c"

compiler_flags="-g -fPIC -MD -std=gnu99 -fdiagnostics-absolute-paths -fdeclspec -fPIC -Wall -Wno-missing-braces"
defines="-DDM_OPENGL"

if ((simd_256)); then
	defines="$defines -DDM_SIMD_256"
	compiler_flags="$compiler_flags -msse4.2 -mavx2"
else
	compiler_flags="$compiler_flags -msse4.1"
fi

if ((debug)); then
	defines="-DDM_DEBUG $defines"
	compiler_flags="-O0 $compiler_flags"
else
	compiler_flags="-O3 $compiler_flags"
fi

include_flags="-I$SRC_DIR/ -I$SRC_DIR/lib/ -I$SRC_DIR/lib/mt19937/include -I$SRC_DIR/lib/glad/include"

linker_flags="-g -lX11 -lX11-xcb -lxcb -lxkbcommon -lGL -L/usr/X11R6/lib -lm -ldl"

echo "Building $output..."
clang $c_files $compiler_flags -o $output $defines $include_flags $linker_flags