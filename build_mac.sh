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

objc_files="$SRC_DIR/DarkMatter/impl/platform/dm_platform_mac.m $SRC_DIR/DarkMatter/impl/render/dm_renderer_metal.m"

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
clang $c_files $external_files $objc_files $compiler_flags -o $output $defines $include_flags $linker_flags

cd ..
# shaders
mkdir -p build/assets/shaders/

xcrun -sdk macosx metal $SRC_DIR/DarkMatter/assets/shaders/debug_render.metal -c -o $SRC_DIR/build/assets/shaders/debug_render.air
xcrun -sdk macosx metallib $SRC_DIR/build/assets/shaders/debug_render.air -o $SRC_DIR/build/assets/shaders/debug_render.metallib

xcrun -sdk macosx metal $SRC_DIR/DarkMatter/assets/shaders/imgui.metal -c -o $SRC_DIR/build/assets/shaders/imgui.air
xcrun -sdk macosx metallib $SRC_DIR/build/assets/shaders/imgui.air -o $SRC_DIR/build/assets/shaders/imgui.metallib

xcrun -sdk macosx metal $SRC_DIR/assets/shaders/persp.metal -c -o $SRC_DIR/build/assets/shaders/persp.air
xcrun -sdk macosx metallib $SRC_DIR/build/assets/shaders/persp.air -o $SRC_DIR/build/assets/shaders/persp.metallib

xcrun -sdk macosx metal $SRC_DIR/assets/shaders/light_src.metal -c -o $SRC_DIR/build/assets/shaders/light_src.air
xcrun -sdk macosx metallib $SRC_DIR/build/assets/shaders/light_src.air -o $SRC_DIR/build/assets/shaders/light_src.metallib

xcrun -sdk macosx metal $SRC_DIR/assets/shaders/blackbody.metal -c -o $SRC_DIR/build/assets/shaders/blackbody.air
xcrun -sdk macosx metallib $SRC_DIR/build/assets/shaders/blackbody.air -o $SRC_DIR/build/assets/shaders/blackbody.metallib

xcrun -sdk macosx metal $SRC_DIR/assets/shaders/airy_point.metal -c -o $SRC_DIR/build/assets/shaders/airy_point.air
xcrun -sdk macosx metallib $SRC_DIR/build/assets/shaders/airy_point.air -o $SRC_DIR/build/assets/shaders/airy_point.metallib

# other assets
mkdir -p build/assets/textures
echo "Copying over textures..."
cp -r $SRC_DIR/DarkMatter/assets/textures build/assets/
#cp -r $SRC_DIR/assets/textures build/assets/

mkdir -p build/assets/fonts
echo "Copying over fonts..."
cp -r $SRC_DIR/DarkMatter/assets/fonts build/assets/
#cp -r $SRC_DIR/assets/textures build/assets/