cmake_minimum_required(VERSION 3.20)
set(CMAKE_C_STANDARD 99)

project(MacDebug)

set(SOURCES
    main.c

    dm_platform_mac.m
    dm_renderer_metal.m
)
add_executable(MacDebug ${SOURCES})
target_include_directories(MacDebug
    PUBLIC
        lib/
)

find_library(APPLE_FWK_COCOA Cocoa REQUIRED)
find_library(APPLE_FWK_METAL Metal REQUIRED)
find_library(APPLE_FWK_QUARTZ_CORE QuartzCore REQUIRED)
find_library(APPLE_FWK_FOUNDATION Foundation REQUIRED)
target_link_libraries(MacDebug PUBLIC ${APPLE_FWK_COCOA} ${APPLE_FWK_METAL} ${APPLE_FWK_QUARTZ_CORE} ${APPLE_FWK_FOUNDATION})