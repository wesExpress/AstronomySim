#!/bin/bash

SRC_DIR=$PWD
DM_DIR=$SRC_DIR/DarkMatter

mkdir -p $SRC_DIR/build/assets/shaders/

cd assets/shaders
for file in *.metal; do
	root=${file%.*}
	shader_type=${root: -6}
	output_air=$root.air
	output_metallib=$root.metallib

	echo "Compiling shader: $file"
	xcrun -sdk macosx metal    -gline-tables-only -MO $file -c -o $output_air
	xcrun -sdk macosx metallib $output_air -o $output_metallib

	mv $output_air      $SRC_DIR/build/assets/shaders
	mv $output_metallib $SRC_DIR/build/assets/shaders
done

cd ../..

mkdir -p $SRC_DIR/build/assets/textures
cp "assets/textures/default_texture.png" $SRC_DIR/build/assets/textures

mkdir -p $SRC_DIR/build/assets/fonts
cp "assets/fonts/Chicago.ttf" $SRC_DIR/build/assets/fonts
