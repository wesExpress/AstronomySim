#! /bin/bash

set echo on
output="AstronomySim"

vulkan=0
debug=1
simd_256=1

SRC_DIR=$PWD
DM_DIR=$SRC_DIR/DarkMatter

mkdir -p build
cd build

c_files="$SRC_DIR/app/app.c $SRC_DIR/app/components.c $SRC_DIR/app/camera.c $SRC_DIR/rendering/render_pass.c $SRC_DIR/rendering/debug_render_pass.c $SRC_DIR/rendering/imgui_render_pass.c $SRC_DIR/systems/physics_system.c $SRC_DIR/systems/gravity_system.c"

dm_files="$DM_DIR/dm_impl.c $DM_DIR/platform/dm_platform_linux.c $DM_DIR/dm_physics.c"

if ((vulkan)); then
	dm_files="$dm_files $DM_DIR/rendering/dm_renderer_vulkan.c"
	defines="-DDM_VULKAN"
else
	dm_files="$dm_files $DM_DIR/rendering/dm_renderer_opengl.c $DM_DIR/lib/glad/src/glad.c"
	defines="-DDM_OPENGL"
fi

compiler_flags="-g -MD -std=gnu99 -fPIC -Wall -Wuninitialized -Wno-missing-braces"

if ((simd_256)); then
	defines="$defines -DDM_SIMD_256"
	compiler_flags="$compiler_flags -msse4.2 -mavx2 -mfma"
else
	compiler_flags="$compiler_flags -msse4.1"
fi

if ((debug)); then
	defines="$defines -DDM_DEBUG"
	compiler_flags="$compiler_flags -O0"
	#compiler_flags="$compiler_flags -fsanitize=address"
else
	compiler_flags="$compiler_flags -O2"
fi

include_flags="-I$SRC_DIR/ -I$DM_DIR -I$DM_DIR/lib"
if ((vulkan)); then
	include_flags="$include_flags -I$VULKAN_SDK/Include"
else
	include_flags="$include_flags -I$DM_DIR/lib/glad/include"
fi

linker_flags="-lX11 -lX11-xcb -lxcb -lxkbcommon -L/usr/X11R6/lib -lm "
if ((vulkan)); then
	linker_flags="$linker_flags -L$VULKAN_SDK/Lib -lvulkan"
else
	linker_flags="$linker_flags -lGL"
fi

echo "Building $output..."
gcc $compiler_flags $c_files $dm_files -o $output $defines $include_flags $linker_flags

cd ..

# move assets
mkdir -p build/assets/shaders

cd assets/shaders
for file in *.glsl; do
	if((vulkan)); then
		root=${file%.*}
		shader_type=${root: -5}
		output=$root.spv
		echo "Compiling shader: $file"
		if [[ "$shader_type" == "pixel" ]]; then
			shader_flags=-fshader-stage=frag
		else
			shader_flags=-fshader-stage=vert
		fi
		$VULKAN_SDK/bin/glslc $shader_flags $file -o $output
		mv $output $SRC_DIR/build/assets/shaders
	else
		echo $file
		cp $file $SRC_DIR/build/assets/shaders
	fi
done

cd ../..

mkdir -p $SRC_DIR/build/assets/textures
cp "assets/textures/default_texture.png" $SRC_DIR/build/assets/textures

mkdir -p $SRC_DIR/build/assets/fonts
cp "assets/fonts/Chicago.ttf" $SRC_DIR/build/assets/fonts