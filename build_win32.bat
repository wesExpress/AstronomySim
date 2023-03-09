@echo off
SetLocal EnableDelayedExpansion

SET SRC_DIR=%cd%

SET /A opengl=1
SET /A debug=1
SET /A simd_256=1

SET c_filenames=
FOR /R %%f IN (*.c) do (
	SET c_filenames=!c_filenames! %%f
	ECHO %%f
)

SET linker_flags=/link user32.lib gdi32.lib
SET include_flags=/I%SRC_DIR%\DarkMatter\lib /I%SRC_DIR%\DarkMatter\lib\mt19937\include
SET compiler_flags=/arch:AVX2 /Wall /WL

IF /I "%simd_256%" EQU "1" (
	SET defines="/DDM_SIMD_256"
)

IF /I "%debug%" EQU "1" (
	SET defines=%defines% /DDM_DEBUG
	SET compiler_flags=/W2 /Z7
) ELSE (
	SET defines=%defines% /DDM_RELEASE
	SET compiler_flags=/W2 /Zo /Ox
)

IF /I "%opengl%" EQU "1" (
	SET include_flags=%include_flags% /I%SRC_DIR%\DarkMatter\lib\glad\include
	SET linker_flags=%linker_flags% Opengl32.lib
	SET defines=%defines% /DDM_OPENGL
) ELSE (
	SET c_filenames=!c_filenames:%cd%\DarkMatter\lib\glad\src\glad.c=!
	SET c_filenames=!c_filenames:%cd%\DarkMatter\lib\glad\src\glad_wgl.c=!
	SET linker_flags=%linker_flags% d3d11.lib dxgi.lib dxguid.lib d3dcompiler.lib
)

SET assembly=app

if not exist "build" mkdir build
cd build
ECHO Building %assembly%...
cl %compiler_flags% %defines% /FC %include_flags% %c_filenames% /Fe%assembly% %linker_flags%

REM hlsl shaders
IF /I "%opengl%" EQU "0" (
	SET fxc_flags=/Fd /Od /Zi
	
	if not exist "assets/shaders" mkdir assets\shaders

	REM  app shaders
	cd ../assets/shaders
	
	FOR /R %%f IN (*.hlsl) DO (
		SET fname=%%f
		SET root=!fname:~0,-5!
		SET output=!root!.fxc
		SET shader_type=!root:~-5!

		ECHO Compiling shader: !fname!
		IF /I "!shader_type!" EQU "pixel" (
			SET shader_flags=/E p_main /T ps_5_0
		) ELSE (
			SET shader_flags=/E v_main /T vs_5_0
		)
		ECHO !shader_flags!

		fxc %fxc_flags% !shader_flags! !fname! /Fo !output!

		move !output! ../../build/assets/shaders
	)

	REM dark matter default shaders
	cd ../../DarkMatter/assets/shaders
	
	FOR /R %%f IN (*.hlsl) DO (
		SET fname=%%f
		SET root=!fname:~0,-5!
		SET output=!root!.fxc
		SET shader_type=!root:~-5!

		ECHO Compiling shader: !fname!
		IF /I "!shader_type!" EQU "pixel" (
			SET shader_flags=/E p_main /T ps_5_0
		) ELSE (
			SET shader_flags=/E v_main /T vs_5_0
		)
		ECHO !shader_flags!

		fxc %fxc_flags% !shader_flags! !fname! /Fo !output!

		move !output! ../../../build/assets/shaders
	)

	cd ../../..
) ELSE (
	
	if not exist "assets/shaders" mkdir assets\shaders
	
	cd ../assets/shaders
	FOR /R %%f IN (*.glsl) DO (
		copy /y "%%f" ..\..\build\assets\shaders\
	)

	cd ../../DarkMatter/assets/shaders
	FOR /R %%f IN (*.glsl) DO (
		copy /y "%%f" ..\..\..\build\assets\shaders\
	)

	cd ../../..
)

if not exist "build/assets" mkdir build\assets

if not exist "build/assets/textures" mkdir build\assets\textures
xcopy /s /y DarkMatter\assets\textures\ build\assets\textures\

if not exist "build/assets/fonts" mkdir build\assets\fonts
xcopy /s /y DarkMatter\assets\fonts\ build\assets\fonts\
