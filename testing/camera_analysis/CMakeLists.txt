cmake_minimum_required(VERSION 3.16)
project(trashcan_tracker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ── Automatically detect if we are on actual Raspberry Pi hardware ───────────
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
  add_compile_options(-O3 -mcpu=native -ffast-math)
  set(IS_RASPBERRY_PI TRUE)
else()
  add_compile_options(-O2)
  set(IS_RASPBERRY_PI FALSE)
endif()

# ── Find OpenCV ───────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS core imgproc video videoio highgui)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# ── Only look for libcamera if compiling directly on the Pi ───────────────────
if(IS_RASPBERRY_PI)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBCAMERA REQUIRED libcamera)
else()
    # On WSL/Windows, define a flag to use standard OpenCV camera streams instead
    add_compile_definitions(USING_OPENCV_CAM)
endif()

# ── Sources ───────────────────────────────────────────────────────────────────
set(SOURCES
    src/main.cpp
    src/camera.cpp
    src/detector.cpp
    src/motor_controller.cpp
)

add_executable(trashcan ${SOURCES})
target_include_directories(trashcan PRIVATE include)

if(IS_RASPBERRY_PI)
    target_include_directories(trashcan PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
    target_link_libraries(trashcan PRIVATE lccv ${LIBCAMERA_LIBRARIES} ${OpenCV_LIBS})
    target_link_options(trashcan PRIVATE -Wl,--allow-shlib-undefined)
else()
    target_link_libraries(trashcan PRIVATE ${OpenCV_LIBS})
endif()

install(TARGETS trashcan DESTINATION bin)