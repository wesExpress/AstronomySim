#! /bin/bash

set echo on

vulkan=0

SRC_DIR=$PWD
DM_DIR=$SRC_DIR/DarkMatter

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

cd DarkMatter/assets/shaders
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

cd ../../..

mkdir -p $SRC_DIR/build/assets/textures
cp -r "assets/textures/" $SRC_DIR/build/assets

mkdir -p $SRC_DIR/build/assets/fonts
cp -r "assets/fonts/" $SRC_DIR/build/assets

mkdir -p $SRC_DIR/build/assets/models
cp -r "assets/models/" $SRC_DIR/build/assets