# sai-kinova

Repository for controlling the Kinova Mico or Jaco via SAI 2.0. We have derived this repository from the CS225a (experimental robotics) class repository.

## Automatic Installation Instructions (for Ubuntu and Mac only)
1. If you have a Mac, install Brew (https://brew.sh/). If you have Ubuntu, install git (sudo apt install git)

2. Clone this repository

   ```git clone https://github.com/StanfordVL/sai-kinova.git sai-kinova.git```

3. Obtain sai2-simulation.zip and extract its contents into sai-kinova.git/sai2-simulation.

4. Run the install script inside sai-kinova.git. This will take a few minutes.

   ```sh install.sh```

5. Build the sai-kinova.git applications

   ```sh make.sh```

## Manual Installation Instructions (for any Linux distro or Mac)
1. Install Brew for Mac (https://brew.sh/) or have Ubuntu installed or any other distro with the following packages. Also install git (sudo apt-get install git). Windows will have to dualboot ubuntu or install a VM.

2. Install Cmake

   Linux: ```sudo apt-get install cmake```
   
   Mac: ```brew install cmake```
   
3. Install Eigen

   Linux:```sudo apt-get install libeigen3-dev```
    
   Mac:```brew install eigen```
    
4. Install TinyXML
 
   Linux:```sudo apt-get install libtinyxml2-dev```
    
   Mac:```brew install tinyxml2```

5. Install JsonCPP

   Linux:```sudo apt-get install libjsoncpp-dev```
    
   Mac:```brew install jsoncpp```
   
6. Install HiRedis Client

   Linux:```sudo apt-get install libhiredis-dev```
    
   Mac:```brew install hiredis```
   
7. Install GLFW + Other things Chai3D Needs

   Linux:
   ```
   sudo apt-get install libglfw3-dev
   sudo apt-get install xorg-dev
   sudo apt-get install freeglut3-dev
   sudo apt-get install libasound2-dev
   sudo apt-get install libusb-1.0-0-dev
   ```
    
   Mac:
   ```
   brew install glfw3
   brew install libusb
   ```
   
8. Install Redis-Server

   Linux:```sudo apt-get install redis-server```
    
   Mac:```brew install redis```
   
9. Install Yaml-CPP

   Linux:```sudo apt-get install libyaml-cpp-dev```
    
   Mac:```brew install yaml```
   
10. Clone, Build, Install RBDL
    Download a copy of the repository as a zip from https://bitbucket.org/rbdl/rbdl
    Extract into a folder (should be called rbdl)
    ```
    cd rbdl
    mkdir build 
    cd build
    cmake -DRBDL_BUILD_ADDON_URDFREADER=On -DRBDL_USE_ROS_URDF_LIBRARY=OFF ..
    make -j4
    sudo make install
    ```
   
11. Install Chai

    Download and extract the multiplatform release from : http://www.chai3d.org/download/releases
    Cd into the extracted folder
 
    ```
    mkdir build
    cd build
    cmake ..
    make -j4
    ```
    
12. Extract the Zip of SAI2 Simulation Library given to you and cd into the extracted folder.
    ```
    mkdir build
    cd build
    cmake ..
    make -j4
    ```
13. Clone the SAI2 Common library and Build
    ```
    git clone https://github.com/manips-sai/sai2-common.git
    cd sai2-common
v    mkdir build
    cd build
    cmake ..
    make -j4
    ```
    
14. Clone This Repo
    ```
    sh make.sh
    ```

## Post-Installation

1. Inside bin, you will find visualizer, simulator, and controller. Run all of these using one of the following. The shell scripts set the controller gains, initialize starting joint position, and kick off the simulator, visualizer, and controller.
   ```
   sh init_jaco.sh
   sh init_mico.sh
   ```
