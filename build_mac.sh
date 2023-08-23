#!/bin/bash

set echo on
output="app"
debug=1

SRC_DIR=$PWD

mkdir -p build
cd build

c_files="$SRC_DIR/main.c $SRC_DIR/app.c $SRC_DIR/render_pass.c"

dm_files="$SRC_DIR/dm_impl.c $SRC_DIR/dm_physics.c"
objc_files="$SRC_DIR/dm_platform_mac.m $SRC_DIR/dm_renderer_metal.m"

compiler_flags="-g -fPIC -MD -std=gnu99 -fdiagnostics-absolute-paths -fPIC -Wall -Wno-missing-braces"

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

include_flags="-I$SRC_DIR -I$SRC_DIR/lib/"

linker_flags="-g -framework Cocoa -lobjc -framework QuartzCore -framework CoreFoundation -framework Metal"

echo "Building $output..."
gcc $c_files $dm_files $objc_files $compiler_flags -o $output $defines $include_flags $linker_flags

cd ..
# shaders
mkdir -p build/assets/shaders/

xcrun -sdk macosx metal -gline-tables-only -MO $SRC_DIR/test.metal -c -o $SRC_DIR/build/assets/shaders/test.air
xcrun -sdk macosx metallib $SRC_DIR/build/assets/shaders/test.air -o $SRC_DIR/build/assets/shaders/test.metallib
