#!/bin/bash

SRC_DIR=$PWD
DM_DIR=$SRC_DIR/DarkMatter

mkdir -p $SRC_DIR/build/assets/shaders/
mkdir -p $SRC_DIR/bin/Debug/assets/shaders/
mkdir -p $SRC_DIR/bin/Release/assets/shaders/

cd $SRC_DIR/assets/shaders
for file in *.metal; do
	root=${file%.*}
	shader_type=${root: -6}
	output_air=$root.air
	output_metallib=$root.metallib

	echo "Compiling shader: $file"
	xcrun -sdk macosx metal    -gline-tables-only -fvectorize -ffast-math -funroll-loops -MO $file -c -o $output_air
	xcrun -sdk macosx metallib $output_air -o $output_metallib

	cp $output_air      $SRC_DIR/build/assets/shaders
	cp $output_metallib $SRC_DIR/build/assets/shaders

	cp $output_air      $SRC_DIR/bin/Debug/assets/shaders
	cp $output_metallib $SRC_DIR/bin/Debug/assets/shaders

	mv $output_air      $SRC_DIR/bin/Release/assets/shaders
	mv $output_metallib $SRC_DIR/bin/Release/assets/shaders
done

cd $DM_DIR/assets/shaders
for file in *.metal; do
	root=${file%.*}
	shader_type=${root: -6}
	output_air=$root.air
	output_metallib=$root.metallib

	echo "Compiling shader: $file"
	xcrun -sdk macosx metal    -gline-tables-only -ffast-math -fvectorize -MO $file -c -o $output_air
	xcrun -sdk macosx metallib $output_air -o $output_metallib

	cp $output_air      $SRC_DIR/build/assets/shaders
	cp $output_metallib $SRC_DIR/build/assets/shaders

	cp $output_air      $SRC_DIR/bin/Debug/assets/shaders
	cp $output_metallib $SRC_DIR/bin/Debug/assets/shaders

	mv $output_air      $SRC_DIR/bin/Release/assets/shaders
	mv $output_metallib $SRC_DIR/bin/Release/assets/shaders
done

cd ../../..

mkdir -p $SRC_DIR/build/assets/textures
cp -R "assets/textures" $SRC_DIR/build/assets/
cp -R "assets/textures" $SRC_DIR/bin/Debug/assets/
cp -R "assets/textures" $SRC_DIR/bin/Release/assets/

mkdir -p $SRC_DIR/build/assets/fonts
cp -R "assets/fonts" $SRC_DIR/build/assets/
cp -R "assets/fonts" $SRC_DIR/bin/Debug/assets/
cp -R "assets/fonts" $SRC_DIR/bin/Release/assets/

mkdir -p $SRC_DIR/build/assets/models
mkdir -p $SRC_DIR/bin/Debug/assets/models
mkdir -p $SRC_DIR/bin/Release/assets/models
cp "assets/models/Planet_2.glb" $SRC_DIR/build/assets/models/
cp "assets/models/Planet_2.glb" $SRC_DIR/bin/Debug/assets/models
cp "assets/models/Planet_2.glb" $SRC_DIR/bin/Release/assets/models