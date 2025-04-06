cmake_minimum_required(VERSION 3.12)
set(CMAKE_TOOLCHAIN_FILE "E:/ENVs/vcpkgs/vcpkg-export/scripts/buildsystems/vcpkg.cmake")
project(DEMO LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

add_definitions("-DNOMINMAX")

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W0")
endif(MSVC)

# set(MP_INCLUDE_DIR  E:/Pro/github/MPlib-windows/include)
# set(MP_LIB  E:/Pro/github/MPlib-windows/install/lib/mp.lib)
set(PYTHON_EXECUTABLE E:/ENVs/pymplib/python.exe)
add_subdirectory(pybind11)
find_package(Eigen3 3.4.0 REQUIRED)
# set(Boost_NO_WARN_NEW_VERSIONS 1)  # silence Boost CMake warnings
# find_package(Boost REQUIRED COMPONENTS system filesystem)
# find_package(ompl REQUIRED)
# find_package(fcl REQUIRED)
# find_package(pinocchio REQUIRED)
# find_package(assimp REQUIRED)
# find_package(orocos_kdl REQUIRED)
# find_package(urdfdom REQUIRED)
# find_package(toppra REQUIRED)

# set(Torch_DIR E:/ENVs/libtorch-2.6.0+cu124/share/cmake/Torch)
# find_package(Torch REQUIRED)

# store libries in a variable
set(LIBS #ompl fcl assimp::assimp orocos-kdl Boost::system Boost::filesystem urdfdom::urdfdom_model urdfdom::urdfdom_world
# ${MP_LIB} toppra::toppra ${TORCH_LIBRARIES}
pybind11::embed
)
# compile test_articulated_model and run the test
add_executable(demo "demo.cpp")

# target_include_directories(demo PRIVATE ${OMPL_INCLUDE_DIRS}
# ${pinocchio_INCLUDE_DIRS}
# ${urdfdom_INCLUDE_DIRS}/urdfdom
# ${urdfdom_headers_INCLUDE_DIRS}/urdfdom_headers
# ${TORCH_INCLUDE_DIRS}
# ${MP_INCLUDE_DIR}
# )

target_link_libraries(demo PRIVATE ${LIBS} )






