cmake_minimum_required(VERSION 3.20)
set(CMAKE_C_STANDARD 99)

project(MacDebug)

set(SOURCES
	#app/app.c
	#app/components.c
	#app/raytrace_app.c
	app/camera.c
	app/compute_test.c

	DarkMatter/dm_impl.c
	DarkMatter/dm_physics.c
	DarkMatter/platform/dm_platform_mac.m
	DarkMatter/rendering/dm_renderer_metal.m
	DarkMatter/dm_imgui.c
)
add_executable(MacDebug ${SOURCES})
target_include_directories(MacDebug
    PUBLIC
		DarkMatter/
		DarkMatter/lib/
)

find_library(APPLE_FWK_COCOA Cocoa REQUIRED)
find_library(APPLE_FWK_METAL Metal REQUIRED)
find_library(APPLE_FWK_QUARTZ_CORE QuartzCore REQUIRED)
find_library(APPLE_FWK_FOUNDATION Foundation REQUIRED)
target_link_libraries(MacDebug PUBLIC ${APPLE_FWK_COCOA} ${APPLE_FWK_METAL} ${APPLE_FWK_QUARTZ_CORE} ${APPLE_FWK_FOUNDATION})
