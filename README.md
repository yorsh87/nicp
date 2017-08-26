NICP (Normal Iterative Closest Point)
====

NICP is a novel on-line method to recursively align point clouds. This method
exploits the 3D structure to determine the data association between the two
clouds taking into account each point and its local features of the surface:
normals and curvature.
This method adopt a line of sight criterion to find the corresponding points between
the two clouds to register. This, togheter with the efficient algorithms and
data structures used by NICP, increase the speed of the method allowing
real-time computation.
NICP solves the alignment problem by casting a least squares formulation that
minimizes an error metric depending on both the point coordinates and the
associated normal. This renders the algorithm more robust and accurate, thus
computing better transformation.

Tutorials and much more @ http://goo.gl/W3qXbE
----

System Requirements
----

### Minimum

- CMake  >= 3.5
- Eigen3 >= 3.2.0
- OpenCV >= 2.4.8
- Flann  >= 1.7

### Complete

- OpenGL
- Qt5
- QGLViewer

Building on Linux (Ubuntu 16.04 LTS)
----

$> sudo apt install git cmake libeigen3-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev libflann-dev libopencv-dev freeglut3-dev

$> git clone https://github.com/yorsh87/nicp.git

$> cd nicp

$> mkdir build

$> cd build

$> cmake ..

$> make

Branches
----

- master : current stable branch
- develop: current development branch
- iros2015_experiments: code snapshot used for IROS 2015 paper publication [[pdf](http://jacoposerafin.com/wp-content/uploads/serafin15iros.pdf)]
- ras2016_experiments: code snapshot used for RAS 2017 paper publication [[pdf](http://jacoposerafin.com/wp-content/uploads/serafin17ras.pdf)]

Examples
----

Once you compiled the code you will have the following exmaple binaries:
- nicp_simple_aligner is a binary that, given a set of depth images and a .txt file containing the list of depth images to align, perform the point cloud registrations
- nicp_aligner same as nicp_simple_aligner, but it uses an incremental version of the algorithm
- nicp_aligner_gui is a simple GUI for point cloud alignment, several parameters of the algorithm can be modified with GUI buttons
- nicp_cloud_prop_viewer simple GUI to see basic properties of a depth image / point cloud
- nicp_simple_viewer is a GUI that given a folder allows to visualize all the point clouds, saved in .nicp format, in the folder
