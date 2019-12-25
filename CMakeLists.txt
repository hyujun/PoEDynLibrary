cmake_minimum_required(VERSION 2.8)
project(PoEDynLibrary)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(Base)
add_subdirectory(Control)
add_subdirectory(Dynamics)
add_subdirectory(Kinematics)
add_subdirectory(Model)

add_library(dynlib
        ${PROJECT_SOURCE_DIR}/Base/LieOperator.cpp
        ${PROJECT_SOURCE_DIR}/Control/Controller.cpp
        ${PROJECT_SOURCE_DIR}/Control/Trajectory.cpp
        ${PROJECT_SOURCE_DIR}/Dynamics/LieDynamics.cpp
        ${PROJECT_SOURCE_DIR}/Kinematics/PoEKinematics.cpp
        ${PROJECT_SOURCE_DIR}/Model/PropertyDefinition.cpp
        ${PROJECT_SOURCE_DIR}/Model/SerialManipulator.cpp
        )

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC dynlib)

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
