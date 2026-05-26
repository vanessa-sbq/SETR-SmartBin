cmake_minimum_required(VERSION 3.16)
project(trashcan_tracker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Release build by default (important for Pi performance) ──────────────────
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Optimise hard for the Pi's ARM core when cross-compiling or building on-device
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
  add_compile_options(-O3 -mcpu=native -ffast-math)
else()
  add_compile_options(-O2)
endif()

# ── Find OpenCV ───────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS core imgproc video videoio highgui)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# ── Sources ───────────────────────────────────────────────────────────────────
set(SOURCES
    src/main.cpp
    src/camera.cpp
    src/detector.cpp
    src/motor_controller.cpp
)

add_executable(trashcan ${SOURCES})

target_include_directories(trashcan PRIVATE include)
target_link_libraries(trashcan PRIVATE ${OpenCV_LIBS})
target_link_options(trashcan PRIVATE -Wl,--allow-shlib-undefined)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS trashcan DESTINATION bin)
