# Implementation of [Example-Based Facial Rigging](https://www.hao-li.com/Hao_Li/Hao_Li_-_publications_%5BExample-Based_Facial_Rigging%5D.html)

A method for generating a facial blendshape rig from a source rig and a set of corresponding example poses.
The method alternates between optimizing for the blendshapes, utilizing an approach derived from [Deformation Transfer](#useful-references), and for the blending weights.
The resulting blendshapes carry the source's controller semantics and maintain the identity of the target.

Implemented in C++17

Using:
* [Eigen 3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
* [OpenMesh](https://www.graphics.rwth-aachen.de/software/openmesh/)
* [CXXOpts](https://github.com/jarro2783/cxxopts)

## Build
### Dependencies
Download and install Eigen, OpenMesh, and CXXOpts according to their documentation.

### CMakeLists.txt
Adjustments to CMakeLists.txt may be necessary depending on your environment.
Lines marked with "<<<" are likely candidates.

### Build
To prepare the build environment, create a build directory and run cmake inside of it.
```commandline
mkdir build
cd build
cmake ..
```
To build the Example-Based Facial Rigging, make the 'ebfr' target.
```commandline
make ebfr
```

## Execution
```commandline
ebfr --source-blendshapes "../data/source/blendshapes" --source-poses "../data/source/poses/" --source-weights "../data/source/poses/weights.csv" --target-neutral "../data/target/blendshapes/neutral.obj" --target-poses "../data/target/poses/" --target-weights "../data/target/poses/weights.csv" --output "output"
```
* --source-blendshapes: Path to the directory containing the source blendshape mesh files
  * Blendshapes are expected to be named sequentially from '0' and in OBJ format
* --source-poses: Path to the directory containing the source pose mesh files
  * Pose mesh names are expected to match the names found in the weights file
* --source-weights: Path to the source pose-weights file
  * See Weights CSV below
* --target-neutral: Path to the target neutral mesh file
  * Neutral mesh is expected to be an OBJ file
* --target-poses: Path to the directory containing the target pose mesh files
    * Pose mesh names are expected to match the names found in the weights file
* --target-weights: Path to the target (estimated) pose-weights file
    * See Weights CSV below
* --output Path to a directory to write the final target blendshapes

### Weights CSV
#### Format
**Header Row**

Pose, 0, 1, ..., # of blendshapes

**Row**

PoseName, w0, w1, ..., wN

PoseName should match the name of a mesh in the corresponding poses directory

#####Example
Pose | 0 | 1 | 2 | 3 | 4 
---- | --- | --- | --- | --- | ---
Smile | 0.1 | 0.2 | 0.3 | 0.4 | 0


## Notes
* There must be a one-to-one correspondence between source and target meshes.
* The source and target poses must also correspond to one another.
* All meshes are, at this time, expected to be in OBJ format
* Runtime on a reasonable PC is roughly 50s with 13k vertices, 28 blendshapes, and 10 example poses. 

## Useful References
[Deformation Transfer for Triangle Meshes by Sumner, Popovic. 2004](https://people.csail.mit.edu/sumner/research/deftransfer/) ([Project](https://github.com/kyle-gh/DeformationTransfer))

[Deformation Transfer for Detail-Preserving Surface Editing by Botsch, Sumner, Pauly, and Gross. 2006](https://lgg.epfl.ch/publications/2006/botsch_2006_DTD.pdf)