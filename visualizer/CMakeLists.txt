cmake_minimum_required(VERSION 3.11)

project(visualizer
        DESCRIPTION
        "Focal Path Guiding Simulators and Visualizer"
        LANGUAGES
        CXX C
        )

if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

include("cmake/detect_emscripten.cmake")
include("cmake/imgui.cmake")

set(TARGETS Simulator Visualizer Reibold)
foreach (target IN LISTS TARGETS)
    file(GLOB ${target}_SOURCES CONFIGURE_DEPENDS "include/visualizer/**/*.h" "src/main.cpp" "src/apps/${target}.cpp")

    add_executable(${target} ${${target}_SOURCES})
    target_link_libraries(${target} PRIVATE imgui)
    target_include_directories(${target} PUBLIC ${CMAKE_SOURCE_DIR}/include)
    target_include_directories(${target} PRIVATE ${CMAKE_SOURCE_DIR}/../mitsuba/src/integrators/path)
    target_compile_features(${target} PUBLIC cxx_std_17)

    file(COPY data DESTINATION ${CMAKE_BINARY_DIR})
    file(COPY ${lore_DATA} DESTINATION ${CMAKE_BINARY_DIR})

    if (CMAKE_CXX_COMPILER_ID STREQUAL "Emscripten")
        set_target_properties(
                ${target}
                PROPERTIES
                LINK_FLAGS "--embed-file data -s WASM=1 -s ALLOW_MEMORY_GROWTH=1 -s NO_EXIT_RUNTIME=0 -s EXPORT_ALL=1 -s DISABLE_EXCEPTION_CATCHING=0 -s FORCE_FILESYSTEM=1 -s SINGLE_FILE=1 -s ASSERTIONS=1 -s TOTAL_MEMORY=64MB"
                OUTPUT_NAME "${target}"
                SUFFIX ".html"
        )
    endif ()
endforeach ()
