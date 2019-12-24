cmake_minimum_required(VERSION 2.8)
project(PoEDynLibrary)

set(CMAKE_CXX_STANDARD 14)

include_directories(Base)
include_directories(Control)
include_directories(Dynamics)
include_directories(Kinematics)
include_directories(Model)

add_executable(${PROJECT_NAME} src/main.cpp)

find_package(Eigen3 3.0 REQUIRED NO_MODULES)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

find_package(OpenMP)
if(NOT TARGET OpenMP::OpenMP_CXX)

    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
            PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    set_property(TARGET OpenMP::OpenMP_CXX
            PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

endif()
target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
