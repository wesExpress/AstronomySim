#!/bin/bash

set echo on
output="app"

SRC_DIR=$PWD

mkdir -p build
cd build

c_files="$SRC_DIR/main.c"
c_files="$c_files $(find $SRC_DIR/app -type f -name "*.c")"
c_files="$c_files $(find $SRC_DIR/systems -type f -name "*.c")"
external_files="$SRC_DIR/DarkMatter/lib/mt19937/src/mt19937.c $SRC_DIR/DarkMatter/lib/mt19937/src/mt19937_64.c"

compiler_flags="-g -fPIC -MD -std=gnu99 -fdiagnostics-absolute-paths -fdeclspec -fPIC -Wall -Wno-missing-braces"

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

include_flags="-I$SRC_DIR -I$SRC_DIR/DarkMatter/lib/mt19937/include -I$SRC_DIR/DarkMatter -I$SRC_DIR/DarkMatter/lib"

linker_flags="-g -framework Cocoa -lobjc -framework QuartzCore -framework CoreFoundation -framework Cocoa -framework Metal"

echo "Building $output..."
clang $c_files $external_files $compiler_flags -o $output $defines $include_flags $linker_flags