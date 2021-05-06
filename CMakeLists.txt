cmake_minimum_required(VERSION 3.17)
project(examplebasedfacialrigging)

set(CMAKE_CXX_STANDARD 17)

# additional CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# OpenMesh
set(OPENMESH_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../OpenMesh/lib) # <<<
find_package(OpenMesh REQUIRED)
include_directories(${OPENMESH_INCLUDE_DIRS})

# Eigen
set(ENV{EIGEN3_ROOT} ${CMAKE_CURRENT_SOURCE_DIR}/../eigen) # <<<
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# CXXOpts
set(CXXOPTS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../cxxopts/include) # <<<
include_directories(${CXXOPTS_INCLUDE_DIR})

set_property(
        DIRECTORY
        APPEND PROPERTY COMPILE_DEFINITIONS _USE_MATH_DEFINES
)

set(EBFR_SOURCE src/ebfr/GradientSolver.cpp src/ebfr/GradientSolver.h src/ebfr/Gradients.cpp src/ebfr/Gradients.h src/ebfr/Parameter.h src/ebfr/BlendshapeSolver.cpp src/ebfr/BlendshapeSolver.h src/ebfr/Rig.cpp src/ebfr/Rig.h src/ebfr/SolverBase.cpp src/ebfr/SolverBase.h src/ebfr/VertexSolver.cpp src/ebfr/VertexSolver.h src/ebfr/WeightsSolver.cpp src/ebfr/WeightsSolver.h)
set(SHARED_SOURCE src/shared/CSV.cpp src/shared/CSV.h src/shared/FS.cpp src/shared/FS.h src/shared/Matrix.h src/shared/Mesh.cpp src/shared/Mesh.h src/shared/SolverUtil.cpp src/shared/SolverUtil.h src/shared/Timing.h src/shared/Util.cpp src/shared/Util.h)

set(EBFR_LIBRARIES ${OPENMESH_LIBRARIES} Eigen3::Eigen)
if(APPLE)
    set(EBFR_LIBRARIES ${EBFR_LIBRARIES} "-framework Accelerate" ${CMAKE_DL_LIBS})
endif()

add_executable(ebfr  ${SHARED_SOURCE} ${EBFR_SOURCE} src/main.cpp src/Args.h)
TARGET_LINK_LIBRARIES(ebfr ${EBFR_LIBRARIES})

add_executable(test-gradient ${SHARED_SOURCE} ${EBFR_SOURCE} src/test/gradient.cpp src/Args.h)
TARGET_LINK_LIBRARIES(test-gradient ${EBFR_LIBRARIES})

add_executable(test-vertex ${SHARED_SOURCE} ${EBFR_SOURCE} src/test/vertex.cpp src/Args.h)
TARGET_LINK_LIBRARIES(test-vertex ${EBFR_LIBRARIES})

add_executable(test-weights ${SHARED_SOURCE} ${EBFR_SOURCE} src/test/weights.cpp src/Args.h)
TARGET_LINK_LIBRARIES(test-weights ${EBFR_LIBRARIES})

add_executable(pose-gen src/shared/CSV.cpp src/shared/CSV.h src/shared/FS.cpp src/shared/FS.h src/shared/Matrix.h src/shared/Mesh.cpp src/shared/Mesh.h src/shared/Timing.h src/shared/Util.cpp src/shared/Util.h src/ebfr/Rig.cpp src/ebfr/Rig.h src/test/posegen.cpp)
TARGET_LINK_LIBRARIES(pose-gen ${OPENMESH_LIBRARIES})