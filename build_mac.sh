#!/bin/bash

set echo on
output="AstronomySim"
debug=0

SRC_DIR=$PWD
DM_DIR=$SRC_DIR/DarkMatter

mkdir -p build
cd build

c_files="$SRC_DIR/main.c $SRC_DIR/app/app.c $SRC_DIR/app/components.c $SRC_DIR/app/camera.c $SRC_DIR/rendering/render_pass.c $SRC_DIR/rendering/debug_render_pass.c $SRC_DIR/rendering/imgui_render_pass.c $SRC_DIR/systems/physics_system.c $SRC_DIR/systems/gravity_system.c"
dm_files="$DM_DIR/dm_impl.c $DM_DIR/dm_physics.c"
objc_files="$DM_DIR/platform/dm_platform_mac.m $DM_DIR/rendering/dm_renderer_metal.m"

compiler_flags="-g -fPIC -MD -std=gnu99 -fdiagnostics-absolute-paths -fPIC -Wall -Wno-missing-braces -msse4.2 -mavx2"

if ((debug)); then
	defines="-DDM_DEBUG $defines"
	compiler_flags="-O0 $compiler_flags"
else
	defines="-DDM_RELEASE $defines"
	compiler_flags="-O3 $compiler_flags"
fi

include_flags="-I$SRC_DIR -I$DM_DIR -I$DM_DIR/lib/"

linker_flags="-framework Cocoa -lobjc -framework QuartzCore -framework CoreFoundation -framework Metal"

echo "Building $output..."
gcc $c_files $dm_files $objc_files $compiler_flags -o $output $defines $include_flags $linker_flags
