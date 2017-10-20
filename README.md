# HumanFoot

Simulation system for humanoid walking with segmented foot

It based on python 2.7.

## How to build and run
### In linux,

1. just make! Next lines are deprecated....

1. download and setup boost
	
		$ ./bootstrap.sh
		$ sudo ./b2 install
	
2. setup using apt-get

        sudo apt-get install python-dev python-numpy freeglut3-dev python-fltk libgle3

3. setup using pip

    	pip install --upgrade pip setuptools
    	pip install PyOpenGL PyOpenGL-accelerate cvxopt pyode future pillow

3. build VirtualPhysics2010 and copy libvpLib.a to PyCommon/modules/usr/lib/
4. make in PyCommon/modules (if there is a copy error, edit setup.py in Renderer and Simulator)
5. Run main_MomentumProject.py in MomentumProject/

### In MAC OS X,

first, install brew.

in OSX terminal,

    brew update
    brew install wget
    brew install python
    brew install cmake fltk ode suite-sparse

after make virtual environment with system packages and activate virtuanenv,
    pip install --upgrade pip setuptools
    pip install numpy
    pip install PyOpenGL PyOpenGL-accelerate
    pip install cvxopt
    pip install pyode
    pip install openopt
    pip install cma

setup pyfltk (google and download)
     in python/fltk\_wrap.cpp,
     add void in front of free\_color

     python setup.py build
     python setup.py install

install bullet manually,

for dart setup,
    
    brew install https://raw.githubusercontent.com/Homebrew/homebrew-core/50b94fe634d752985c489243033026a04d74abb6/Formula/boost.rb
    brew install https://raw.githubusercontent.com/Homebrew/homebrew-core/71e39e8462350492fc69040db18f9a555040880d/Formula/boost-python.rb
    brew install eigen assimp homebrew/science/libccd dartsim/dart/fcl open-scene-graph nlopt homebrew/science/ipopt tinyxml tinyxml2 ros/deps/urdfdom doxygen
    # brew install flann
    brew install https://raw.githubusercontent.com/Homebrew/homebrew-science/7fb6d735213383488fe2e7518148fa6e486588fc/flann.rb
    
    git clone https://github.com/dartsim/dart
    cd dart;git checkout tags/v6.1.2;mkdir build

    in CMakeLists.txt,
    ENABLE_OPENMP to OFF

    cd build;cmake ..;make -j;make install

for pydart setup,
    cd PyCommon/externalLibs/pydart2;make

make install_dependencies_mac
make
