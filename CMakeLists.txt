cmake_minimum_required(VERSION 4.3)

project(ur5)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_BUILD_TYPE "Debug")

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/.deps)

set(THIRD_PARTY_DIR ${CMAKE_SOURCE_DIR}/lib)

set(MUJOCO_BUILD_TESTS OFF)
set(CCD_SINGLE ON)

# link
find_program(MOLD_PROGRAM mold)
if (MOLD_PROGRAM)
    add_link_options(-fuse-ld=mold)
else ()
    message(STATUS "mold not found")
endif ()

FetchContent_Declare(
  mujoco
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
  GIT_TAG 3.10.0
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${THIRD_PARTY_DIR}/mujoco")

FetchContent_MakeAvailable(mujoco)

add_executable(ur5)

target_sources(ur5
  PRIVATE  src/main.cpp src/kinematic.cpp
)

target_compile_definitions(mujoco PUBLIC mjUSESINGLE)

target_link_libraries(ur5 PRIVATE mujoco glfw mujoco)

target_include_directories(ur5 PRIVATE src)
