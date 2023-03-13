#! /bin/bash

set echo on
output="app"

debug=0
simd_256=0

SRC_DIR=$PWD

mkdir -p build
cd build

c_files="$SRC_DIR/main.c"
c_files="$c_files $(find $SRC_DIR/app -type f -name "*.c")"
c_files="$c_files $(find $SRC_DIR/systems -type f -name "*.c")"
external_files=$(find $SRC_DIR/DarkMatter/lib/mt19937 -type f -name "*.c")
external_files="$external_files $SRC_DIR/DarkMatter/lib/glad/src/glad.c"

compiler_flags="-g -fPIC -MD -std=gnu99 -fdiagnostics-absolute-paths -fPIC -Wall -Wno-missing-braces"
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
	compiler_flags="-O2 $compiler_flags"
fi

include_flags="-I$SRC_DIR/ -I$SRC_DIR/DarkMatter/ -I$SRC_DIR/DarkMatter/lib/ -I$SRC_DIR/DarkMatter/lib/mt19937/include -I$SRC_DIR/DarkMatter/lib/glad/include"

linker_flags="-g -lX11 -lX11-xcb -lxcb -lxkbcommon -lGL -L/usr/X11R6/lib -lm -ldl"

echo "Building $output..."
clang $c_files $compiler_flags $external_files -o $output $defines $include_flags $linker_flags

cd ..

# move assets
mkdir -p build/assets
cp -r assets/ build
cp -r DarkMatter/assets/ build