## C API

This package lets you use the ZED stereo camera in C .The C API is a wrapper around the ZED SDK which is written in C++ optimized code.

## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com/developers/release/).
- For more information, read the ZED [API documentation](https://www.stereolabs.com/docs/api/index.html).

## Prerequisites

To start using the ZED SDK in C, you will need to install the following dependencies on your system:

- [ZED SDK 5.1](https://www.stereolabs.com/developers/release/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)

## Installing the C API

#### Windows

- Create a "build" folder in the source folder
- Open cmake-gui and select the source and build folders
- Generate the Visual Studio `Win64` solution
- Open the resulting solution
- Build the solution in **Release** mode
- Build **INSTALL**.

#### Linux

Open a terminal in the root directory and execute the following command:
```
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
```


## Use the C api

- Link your project to the sl_zed_c library
- Import the c interface in your code like this :

```
  #include <sl/c_api/zed_interface.h>
```

### Run the tutorials

The [tutorials](https://github.com/stereolabs/zed-examples/tree/master/tutorials) provide simple projects to show how to use each module of the ZED SDK.

## Support

If you need assistance go to our Community site at https://community.stereolabs.com/
