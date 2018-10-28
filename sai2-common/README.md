## Project name: sai2-common (version: 0.1.0)
Note: This project uses Semantic Versioning (http://semver.org/).

### Project description:
The Sai2-Common library implements a set of simplifying interfaces and helper modules to create robot control applications. This includes the following modules:

* __parser__ module: to load a robot and world state specification from xml
* __model__ module: creating articulated rigid body models to represent the robots, 
* __simulation__ module: creating a virtual world with simulated physics, and
* __graphics__ module: rendering the world graphically

### 3rdParty dependencies (* = installation required):

* Redis*: Redis server [brew, apt-get]
* Hiredis*: Redis minimalist client [brew, apt-get]
* JsonCpp*: JSON serialization [brew, apt-get]
* Eigen3*: Linear algebra [brew, apt-get]
* TinyXML2*: XML parser [brew, apt-get]
* GLFW3*: Window management [brew, apt-get]
* Chai3d: Graphics and haptics [https://github.com/chai3d/chai3d]
* RBDL: Articulated rigid-body kinematics, forward dynamics and inverse dynamics. [https://bitbucket.org/rbdl/rbdl/src]  
Note: Please specify the following CMake flags when building RBDL to build the URDF reader addon: 
``` cmake
cmake -DRBDL_BUILD_ADDON_URDFREADER=ON -DRBDL_USE_ROS_URDF_LIBRARY=OFF ..
```

### SAI library dependencies:
* Sai2-Simulation: Articulated rigid-body physics simulation [private at https://github.com/manips-sai/sai2-simulation, contact shameekg@stanford.edu]


### Installation instructions:
-- OSX/ Linux:

1. mkdir build
2. cmake ..
3. make

### Uninstallation instructions: 
-- OSX/ Linux:

1. rm -r build
2. rm -r ~/.cmake/packages/SAI2-COMMON

### Getting started:
Take a look at sample applications under examples/.
You can run them from build/examples/<x-example>/.

### License:
Currently pending licensing. PLEASE DO NOT DISTRIBUTE.

### Project contributors:
* Gerald Brantner
* Brian Soe
* Mikael Jorda
* Shameek Ganguly
* Toki Migimatsu

### For questions, contact:
manips.sai@gmail.com  
mjorda@stanford.edu  
shameekg@stanford.edu
