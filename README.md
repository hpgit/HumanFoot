# HumanFoot

Simulation system for humanoid walking with segmented foot

It based on python 3.6, Boost 1.66, dart v6.1.2.
(python 2.7 is supported also, but will be unsupported soon.)

## How to build and run
### In linux,

1. just make!

        make install_dart_ubuntu
        make install_dependencies
        make

2. Run main_MomentumProject_working.py in MomentumProject/working_example

        cd MomentumProject/working_example
        python main_MomentumProject_working.py

### In MAC OS X,

first, install brew.

in OSX terminal,

    brew update
    brew install python3
    brew install cmake fltk suite-sparse

after make virtual environment with system packages and activate virtuanenv,

    pip install --upgrade pip setuptools
    pip install -r requirements.txt

setup pyfltk (google and download pyfltk-1.*_py3.tar.gz)

     in python/fltk\_wrap.cpp,
     add void in front of free\_color

     python setup.py build
     python setup.py install

for dart setup,
    
    brew install boost
    brew install boost-python3
    brew install bullet eigen assimp flann libccd fcl open-scene-graph nlopt ipopt tinyxml tinyxml2 ros/deps/urdfdom doxygen
    
    git clone https://github.com/dartsim/dart
    cd dart;git checkout tags/v6.1.2;mkdir build
    
    copy FindBoost.cmake to dart/cmake 

    in CMakeLists.txt,
    ENABLE_OPENMP to OFF

    cd build;cmake ..;make -j;make install

for pydart setup,

    make install_dependencies_mac
    make
