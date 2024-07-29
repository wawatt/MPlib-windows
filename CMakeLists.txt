cmake_minimum_required(VERSION 3.12)
project(mp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4244 /wd4305 /wd4819 /wd4267 /MD")
  set(CMAKE_CXX_FLAGS_RELEASE "-Wall -fsized-deallocation")
else(MSVC)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g3 -Wall -Werror -fsized-deallocation -Wno-deprecated-declarations")
endif(MSVC)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# set -fuse-ld=lld if lld is found
find_program(LLD_FOUND ld.lld)
if(LLD_FOUND)
  message(STATUS "Using lld")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=lld")
endif(LLD_FOUND)

# add ccache as compiler launcher
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  message(STATUS "Using ccache")
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
endif(CCACHE_FOUND)

# Pinocchio uses its own FindCppAD, but does not provide it.
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

find_package(Eigen3 3.4.0 REQUIRED)
set(Boost_NO_WARN_NEW_VERSIONS 1)  # silence Boost CMake warnings
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
# find_package(pinocchio REQUIRED)
set(pinocchio_ROOT "E:/ENVs/vcpkgs/pinocchio-2.7.1/install")
set(pinocchio_DIR  " ${pinocchio_DIR}/lib/cmake/pinocchio")
add_definitions("-DNOMINMAX")
set(pinocchio_INCLUDE_DIRS ${pinocchio_ROOT}/include)
set(pinocchio_LIBS ${pinocchio_ROOT}/lib/*)
find_package(assimp REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdfdom REQUIRED)

# store libries in a variable
set(LIBS ompl fcl assimp::assimp orocos-kdl Boost::system Boost::filesystem urdfdom::urdfdom_model urdfdom::urdfdom_world
${pinocchio_LIBS}
)
# mp
file(GLOB_RECURSE MPLIB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library(mp STATIC ${MPLIB_SRC})
target_link_libraries(mp PRIVATE ${LIBS})
target_include_directories(mp PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(mp PUBLIC ${OMPL_INCLUDE_DIRS} #${urdfdom_INCLUDE_DIRS}
${pinocchio_INCLUDE_DIRS}
${urdfdom_INCLUDE_DIRS}/urdfdom
${urdfdom_headers_INCLUDE_DIRS}/urdfdom_headers
)
set_target_properties(mp PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
target_compile_options(mp 
	PRIVATE 
	$<$<CXX_COMPILER_ID:MSVC>:/bigobj> 
	$<$<CXX_COMPILER_ID:GNU>:-Wa,-mbig-obj>) 

if(MSVC)
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(pinocchio_DLL "${pinocchio_ROOT}/bin/pinocchiod.dll")
    message("-----------Debug-------------")
  else()
    set(pinocchio_DLL "${pinocchio_ROOT}/bin/pinocchio.dll")
    message("-----------Release-------------")
  endif()
else(MSVC)
    message(FATAL_ERROR "No actions for other platform.")
endif(MSVC)
ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
    ${pinocchio_DLL}
    $<TARGET_FILE_DIR:${PROJECT_NAME}>
)

# pybind11_mkdoc
file(GLOB_RECURSE MPLIB_MKDOC_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/include/mplib/*.h")
list(FILTER MPLIB_MKDOC_HEADER EXCLUDE REGEX "types.h|macros")
add_custom_target(
  pybind11_mkdoc ALL
  COMMAND "C:/Program Files/Git/bin/bash.exe" "${CMAKE_CURRENT_SOURCE_DIR}/dev/mkdoc.sh"
               "-I$<JOIN:$<TARGET_PROPERTY:mp,INCLUDE_DIRECTORIES>,;-I>"
  BYPRODUCTS "${CMAKE_CURRENT_SOURCE_DIR}/pybind/docstring/*.h"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/dev/mkdoc.sh" "${MPLIB_MKDOC_HEADER}"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMAND_EXPAND_LISTS
  VERBATIM
)

# Pybind11
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third_party/pybind11")
file(GLOB_RECURSE PYBIND_SRC "${CMAKE_CURRENT_SOURCE_DIR}/pybind/*.cpp")
pybind11_add_module(pymp ${PYBIND_SRC})
target_link_libraries(pymp PRIVATE mp)
target_include_directories(pymp PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/pybind")
add_dependencies(pymp pybind11_mkdoc)

# compile test_articulated_model and run the test
add_executable(test_articulated_model "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_articulated_model.cpp")
target_link_libraries(test_articulated_model PRIVATE mp)
add_test(NAME test_articulated_model COMMAND test_articulated_model)
