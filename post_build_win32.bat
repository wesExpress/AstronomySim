@echo off
SetLocal EnableDelayedExpansion

SET SRC_DIR=%cd%
SET DM_DIR=%SRC_DIR%\DarkMatter

REM SET /A vulkan=0
SET /A opengl=0

IF NOT EXIST "build\assets\shaders" mkdir build\assets\shaders

REM shaders
CD assets/shaders
IF /I "%vulkan%" EQU "1" (
	FOR /R %%f IN (*.glsl) DO (
		SET fname=%%f
		SET root=!fname:~0,-5!
		SET output=!root!.spv
		SET shader_type=!root:~-5!

		ECHO Compiling shader: !fname!
		IF /I "!shader_type!" EQU "pixel" (
			SET shader_flags=-fshader-stage=frag
		) ELSE (
			SET shader_flags=-fshader-stage=vert
		)

		%VULKAN_SDK%\bin\glslc !shader_flags! !fname! -o !output!
		MOVE !output! %SRC_DIR%\build\assets\shaders
	)
) ELSE IF /I "%opengl%" EQU "1" (
	FOR /R %%f in (*.glsl) DO (
		COPY /y %%f %SRC_DIR%\build\assets\shaders
	)
) ELSE (
	FOR /R %%f IN (*.hlsl) DO (
		SET fname=%%f
		SET root=!fname:~0,-5!
		SET output=!root!.fxc
		SET shader_type=!root:~-5!
		SET debug_shader=!root!.pdb

		ECHO Compiling shader: !fname!
		IF /I "!shader_type!" EQU "pixel" (
			SET shader_flags=/E p_main /T ps_5_0
		) ELSE IF /I "!shader_type!" EQU "ertex" (
			SET shader_flags=/E v_main /T vs_5_0
		) ELSE IF /I "!shader_type!" EQU "mpute" (
			SET shader_flags=/E c_main /T cs_5_0
		)
		ECHO !shader_flags!

		fxc %fxc_flags% !shader_flags! !fname! /Zi /Fd /Fo !output!

		MOVE !output! %SRC_DIR%\build\assets\shaders
		MOVE !debug_shader! %SRC_DIR%\build\assets\shaders
	)
)

CD ..\..
CD %DM_DIR%\assets\shaders
IF /I "%vulkan%" EQU "1" (
	FOR /R %%f IN (*.glsl) DO (
		SET fname=%%f
		SET root=!fname:~0,-5!
		SET output=!root!.spv
		SET shader_type=!root:~-5!

		ECHO Compiling shader: !fname!
		IF /I "!shader_type!" EQU "pixel" (
			SET shader_flags=-fshader-stage=frag
		) ELSE (
			SET shader_flags=-fshader-stage=vert
		)

		%VULKAN_SDK%\bin\glslc !shader_flags! !fname! -o !output!
		MOVE !output! %SRC_DIR%\build\assets\shaders
	)
) ELSE IF /I "%opengl%" EQU "1" (
	FOR /R %%f in (*.glsl) DO (
		COPY /y %%f %SRC_DIR%\build\assets\shaders
	)
) ELSE (
	FOR /R %%f IN (*.hlsl) DO (
		SET fname=%%f
		SET root=!fname:~0,-5!
		SET output=!root!.fxc
		SET shader_type=!root:~-5!
		SET debug_shader=!root!.pdb

		ECHO Compiling shader: !fname!
		IF /I "!shader_type!" EQU "pixel" (
			SET shader_flags=/E p_main /T ps_5_0
		) ELSE (
			SET shader_flags=/E v_main /T vs_5_0
		)
		ECHO !shader_flags!

		fxc %fxc_flags% !shader_flags! !fname! /Zi /Fd /Fo !output!

		MOVE !output! %SRC_DIR%\build\assets\shaders
		MOVE !debug_shader! %SRC_DIR%\build\assets\shaders
	)
)

CD ..\..\..

IF NOT EXIST "build\assets\textures" mkdir build\assets\textures
COPY /y "assets\textures" build\assets\textures

IF NOT EXIST "build\assets\fonts" mkdir build\assets\fonts
COPY /y "assets\fonts" build\assets\fonts

REM IF NOT EXIST "build\assets\models" mkdir build\assets\models
REM COPY /y "assets\models" build\assets\models

