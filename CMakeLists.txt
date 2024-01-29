cmake_minimum_required(VERSION 3.20)
set(CMAKE_C_STANDARD 99)

project(AstronomySim)

if(APPLE)
	SET(VAR_SOURCES
		DarkMatter/platform/dm_platform_mac.m
		DarkMatter/rendering/dm_renderer_metal.m
	)
elseif(WIN32)
	SET(VAR_SOURCES
		DarkMatter/platform/dm_platform_win32.c
		DarkMatter/rendering/dm_renderer_dx11.c
	)
endif()

set(SOURCES
	app/camera.c
	app/compute_test.c

	app/gravity.c
	app/physics.c
	app/octree.c

	app/default_render.c
	app/debug_render.c
	
	DarkMatter/dm_impl.c
	DarkMatter/dm_physics.c
	DarkMatter/dm_imgui.c
)

add_executable(AstronomySim ${SOURCES} ${VAR_SOURCES})

target_include_directories(AstronomySim
    PUBLIC
		DarkMatter/
		DarkMatter/lib/
)

if(APPLE)
	find_library(APPLE_FWK_COCOA Cocoa REQUIRED)
	find_library(APPLE_FWK_METAL Metal REQUIRED)
	find_library(APPLE_FWK_QUARTZ_CORE QuartzCore REQUIRED)
	find_library(APPLE_FWK_FOUNDATION Foundation REQUIRED)
	
	target_link_libraries(AstronomySim PUBLIC ${APPLE_FWK_COCOA} ${APPLE_FWK_METAL} ${APPLE_FWK_QUARTZ_CORE} ${APPLE_FWK_FOUNDATION})
elseif(WIN32)
	target_link_libraries(AstronomySim d3d11 dxgi dxguid d3dcompiler)
endif()

