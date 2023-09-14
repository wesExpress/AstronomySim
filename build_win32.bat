@echo off
SetLocal EnableDelayedExpansion

SET SRC_DIR=%cd%
SET DM_DIR=%SRC_DIR%\DarkMatter

REM SET /A vulkan=0
SET /A opengl=0
SET /A debug=0
SET /A physics_mt=1

SET c_filenames=%SRC_DIR%\main.c %SRC_DIR%\app\app.c %SRC_DIR%\app\camera.c %SRC_DIR%\app\components.c %SRC_DIR%\rendering\render_pass.c %SRC_DIR%\rendering\debug_render_pass.c %SRC_DIR%\rendering\imgui_render_pass.c %SRC_DIR%\systems\gravity_system.c

IF /I "%physics_mt%" EQU "1" (
	SET c_filenames=%c_filenames% %SRC_DIR%\systems\physics_system_multi_th.c
) ELSE (
	SET c_filenames=%c_filenames% %SRC_DIR%\systems\physics_system.c
)

SET dm_filenames=%DM_DIR%\dm_impl.c %DM_DIR%\platform\dm_platform_win32.c %DM_DIR%\dm_physics.c

SET linker_flags=/link user32.lib gdi32.lib
SET include_flags=/I%SRC_DIR% /I%DM_DIR% /I%DM_DIR%\lib
SET compiler_flags=/arch:AVX512 /Wall /WL /TC /std:c99

IF /I "%debug%" EQU "1" (
	SET defines=%defines% /DDM_DEBUG
	SET compiler_flags=/W2 /Z7 /Od /Ob0 /RTCsu
) ELSE (
	SET defines=%defines% /DDM_RELEASE
	SET compiler_flags=/O2 /Ob3 /Zi
)

IF /I "%vulkan%" EQU "1" (
	SET dm_filenames=%dm_filenames% %DM_DIR%\rendering\dm_renderer_vulkan.c
	SET include_flags=%include_flags% /I%VULKAN_SDK%\Include
	SET linker_flags=%linker_flags% /LIBPATH:%VULKAN_SDK%\Lib vulkan-1.lib
	SET defines=%defines% /DDM_VULKAN
) ELSE IF /I "%opengl%" EQU "1" (
	SET dm_filenames=%dm_filenames% %DM_DIR%\rendering\dm_renderer_opengl.c %DM_DIR%\lib\glad\src\glad.c %DM_DIR%\lib\glad\src\glad_wgl.c
	SET include_flags=%include_flags% /I%DM_DIR%\lib\glad\include
	SET linker_flags=%linker_flags% Opengl32.lib
	SET defines=%defines% /DDM_OPENGL
) ELSE (
	SET dm_filenames=%dm_filenames% %DM_DIR%\rendering\dm_renderer_dx11.c
	SET linker_flags=%linker_flags% d3d11.lib dxgi.lib dxguid.lib d3dcompiler.lib
)

SET assembly=AstronomySim

IF NOT EXIST "build" mkdir build
CD build
ECHO Building %assembly%...
cl %compiler_flags% %defines% /FC %include_flags% %c_filenames% %dm_filenames% /Fe%assembly% %linker_flags%

CD ..
IF NOT EXIST "build\assets\shaders" mkdir build\assets\shaders

cd assets/shaders
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

CD ..\..

IF NOT EXIST "build\assets\textures" mkdir build\assets\textures
COPY /y "assets\textures\default_texture.png" build\assets\textures

IF NOT EXIST "build\assets\fonts" mkdir build\assets\fonts
COPY /y "assets\fonts\Chicago.ttf" build\assets\fonts