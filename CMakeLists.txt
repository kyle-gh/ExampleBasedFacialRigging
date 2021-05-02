cmake_minimum_required(VERSION 3.17)
project(examplebasedfacialrigging)

set(CMAKE_CXX_STANDARD 17)

# additional CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# OpenMesh
set(OPENMESH_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../OpenMesh/lib)
find_package(OpenMesh REQUIRED)
include_directories(${OPENMESH_INCLUDE_DIRS})

set(ENV{EIGEN3_ROOT} ${CMAKE_CURRENT_SOURCE_DIR}/../eigen)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CXXOPTS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../cxxopts/include)
include_directories(${CXXOPTS_INCLUDE_DIR})

set_property(
        DIRECTORY
        APPEND PROPERTY COMPILE_DEFINITIONS _USE_MATH_DEFINES
)

add_executable(ebfr src/shared/CSV.cpp src/shared/CSV.h src/shared/FS.cpp src/shared/FS.h src/shared/Matrix.h src/shared/Mesh.cpp src/shared/Mesh.h src/shared/SolverUtil.cpp src/shared/SolverUtil.h src/shared/Timing.h src/shared/Util.cpp src/shared/Util.h src/ebfr/GradientSolver.cpp src/ebfr/GradientSolver.h src/ebfr/Gradients.cpp src/ebfr/Gradients.h src/ebfr/Parameter.h src/ebfr/BlendshapeSolver.cpp src/ebfr/BlendshapeSolver.h src/ebfr/Rig.cpp src/ebfr/Rig.h src/ebfr/SolverBase.cpp src/ebfr/SolverBase.h src/ebfr/VertexSolver.cpp src/ebfr/VertexSolver.h src/ebfr/WeightsSolver.cpp src/ebfr/WeightsSolver.h src/main.cpp src/Args.h)
TARGET_LINK_LIBRARIES(ebfr "-framework Accelerate" ${OPENMESH_LIBRARIES} ${CMAKE_DL_LIBS})

add_executable(test-gradient src/shared/CSV.cpp src/shared/CSV.h src/shared/FS.cpp src/shared/FS.h src/shared/Matrix.h src/shared/Mesh.cpp src/shared/Mesh.h src/shared/SolverUtil.cpp src/shared/SolverUtil.h src/shared/Timing.h src/shared/Util.cpp src/shared/Util.h src/ebfr/GradientSolver.cpp src/ebfr/GradientSolver.h src/ebfr/Gradients.cpp src/ebfr/Gradients.h src/ebfr/Parameter.h src/ebfr/BlendshapeSolver.cpp src/ebfr/BlendshapeSolver.h src/ebfr/Rig.cpp src/ebfr/Rig.h src/ebfr/SolverBase.cpp src/ebfr/SolverBase.h src/ebfr/VertexSolver.cpp src/ebfr/VertexSolver.h src/ebfr/WeightsSolver.cpp src/ebfr/WeightsSolver.h src/test/gradient.cpp src/Args.h)
TARGET_LINK_LIBRARIES(test-gradient "-framework Accelerate" ${OPENMESH_LIBRARIES} ${CMAKE_DL_LIBS})

add_executable(test-vertex src/shared/CSV.cpp src/shared/CSV.h src/shared/FS.cpp src/shared/FS.h src/shared/Matrix.h src/shared/Mesh.cpp src/shared/Mesh.h src/shared/SolverUtil.cpp src/shared/SolverUtil.h src/shared/Timing.h src/shared/Util.cpp src/shared/Util.h src/ebfr/GradientSolver.cpp src/ebfr/GradientSolver.h src/ebfr/Gradients.cpp src/ebfr/Gradients.h src/ebfr/Parameter.h src/ebfr/BlendshapeSolver.cpp src/ebfr/BlendshapeSolver.h src/ebfr/Rig.cpp src/ebfr/Rig.h src/ebfr/SolverBase.cpp src/ebfr/SolverBase.h src/ebfr/VertexSolver.cpp src/ebfr/VertexSolver.h src/ebfr/WeightsSolver.cpp src/ebfr/WeightsSolver.h src/test/vertex.cpp src/Args.h)
TARGET_LINK_LIBRARIES(test-vertex "-framework Accelerate" ${OPENMESH_LIBRARIES} ${CMAKE_DL_LIBS})

add_executable(test-weights src/shared/CSV.cpp src/shared/CSV.h src/shared/FS.cpp src/shared/FS.h src/shared/Matrix.h src/shared/Mesh.cpp src/shared/Mesh.h src/shared/SolverUtil.cpp src/shared/SolverUtil.h src/shared/Timing.h src/shared/Util.cpp src/shared/Util.h src/ebfr/GradientSolver.cpp src/ebfr/GradientSolver.h src/ebfr/Gradients.cpp src/ebfr/Gradients.h src/ebfr/Parameter.h src/ebfr/BlendshapeSolver.cpp src/ebfr/BlendshapeSolver.h src/ebfr/Rig.cpp src/ebfr/Rig.h src/ebfr/SolverBase.cpp src/ebfr/SolverBase.h src/ebfr/VertexSolver.cpp src/ebfr/VertexSolver.h src/ebfr/WeightsSolver.cpp src/ebfr/WeightsSolver.h src/test/weights.cpp src/Args.h)
TARGET_LINK_LIBRARIES(test-weights "-framework Accelerate" ${OPENMESH_LIBRARIES} ${CMAKE_DL_LIBS})

add_executable(pose-gen src/shared/CSV.cpp src/shared/CSV.h src/shared/FS.cpp src/shared/FS.h src/shared/Matrix.h src/shared/Mesh.cpp src/shared/Mesh.h src/shared/SolverUtil.cpp src/shared/SolverUtil.h src/shared/Timing.h src/shared/Util.cpp src/shared/Util.h src/ebfr/GradientSolver.cpp src/ebfr/GradientSolver.h src/ebfr/Gradients.cpp src/ebfr/Gradients.h src/ebfr/Parameter.h src/ebfr/BlendshapeSolver.cpp src/ebfr/BlendshapeSolver.h src/ebfr/Rig.cpp src/ebfr/Rig.h src/ebfr/SolverBase.cpp src/ebfr/SolverBase.h src/ebfr/VertexSolver.cpp src/ebfr/VertexSolver.h src/ebfr/WeightsSolver.cpp src/ebfr/WeightsSolver.h src/test/posegen.cpp src/Args.h)
TARGET_LINK_LIBRARIES(pose-gen "-framework Accelerate" ${OPENMESH_LIBRARIES} ${CMAKE_DL_LIBS})