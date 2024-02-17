@echo off
SetLocal EnableDelayedExpansion

SET SRC_DIR=%cd%
SET DM_DIR=%SRC_DIR%\DarkMatter

SET /A debug=1
SET /A dx12=1
REM SET /A vulkan=0
SET /A opengl=0
SET /A physics_mt=0
SET /A physics_debug=0
SET /A math_tests=0

REM SET app=%SRC_DIR%\app\raytrace_app.c
REM SET app=%SRC_DIR%\app\compute_test.c %SRC_DIR%\app\octree.c %SRC_DIR%\app\gravity.c %SRC_DIR%\app\physics.c
SET app=%SRC_DIR%\app\render_test.c
REM SET rendering=%SRC_DIR%\app\default_render.c %SRC_DIR%\app\debug_render.c

SET c_filenames=%app% %rendering% %SRC_DIR%\app\camera.c

IF /I "%physics_mt%" EQU "1" (
	REM SET c_filenames=%c_filenames% %SRC_DIR%\systems\physics_system_multi_th.c
) ELSE (
	REM SET c_filenames=%c_filenames% %SRC_DIR%\systems\physics_system.c
)

SET dm_filenames=%DM_DIR%\dm_impl.c %DM_DIR%\platform\dm_platform_win32.c %DM_DIR%\dm_physics.c %DM_DIR%\dm_imgui.c

SET linker_flags=/link user32.lib gdi32.lib
SET include_flags=/I%SRC_DIR% /I%DM_DIR% /I%DM_DIR%\lib /I%DM_DIR%\lib\cglm\include
SET compiler_flags=/arch:AVX512 /Wall /WL /TC /std:c99 /GS-

IF /I "%debug%" EQU "1" (
	SET defines=%defines% /DDM_DEBUG
	SET compiler_flags=/W2 /Z7 /Od /Ob0
) ELSE (
	SET defines=%defines% /DDM_RELEASE
	SET compiler_flags=/O2 /Ob3 /Zi
)

IF /I "%physics_debug%" EQU "1" (
	SET defines=%defines% /DDM_PHYSICS_DEBUG
)

IF /I "%math_tests%" EQU "1" (
	SET defines=%defines% /DDM_MATH_TESTS
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
) ELSE IF /I "%dx12%" EQU "1" (
	SET dm_filenames=%dm_filenames% %DM_DIR%\rendering\dm_renderer_dx12.c
	SET linker_flags=%linker_flags% d3d12.lib dxgi.lib %dxguid.lib d3dcompiler.lib 
	SET defines=%defines% /DDM_DIRECTX12
) ELSE (
	SET dm_filenames=%dm_filenames% %DM_DIR%\rendering\dm_renderer_dx11.c
	SET linker_flags=%linker_flags% d3d11.lib dxgi.lib dxguid.lib d3dcompiler.lib
	SET defines=%defines% /DDM_DIRECTX11
)

SET assembly=AstronomySim

IF NOT EXIST "build" mkdir build
CD build
ECHO Building %assembly%...
cl %compiler_flags% %defines% /FC %include_flags% %c_filenames% %dm_filenames% /Fe%assembly% %linker_flags%
