cmake_minimum_required(VERSION 3.5)

# --- Project ---
set(PROJECT_NAME "secure-comms-prototype")

# --- ESP-IDF ---
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(${PROJECT_NAME})

# --- Gather all source files ---
file(GLOB_RECURSE ALL_C_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/main/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/**/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/**/*.h"
)

# --- Format Target ---
if(NOT CLANG_FORMAT_BIN)
    find_program(CLANG_FORMAT_BIN clang-format REQUIRED)
endif()

if(ALL_C_SOURCES)
    add_custom_target(format
        COMMAND ${CLANG_FORMAT_BIN} -i -style=file ${ALL_C_SOURCES}
        COMMENT "Running clang-format on source/header files"
    )
else()
    message(WARNING "No source/header files found for format target!")
endif()