# HumanFoot

Simulation system for humanoid walking with segmented foot

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

in OSX terminal,

    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
    brew update
    brew install python
    brew install boost --with-python
    brew install boost-python
    brew install fltk
        in El capitan,
        brew reinstall --devel https://raw.githubusercontent.com/dpo/homebrew/ec46018128dde5bf466b013a6c7086d0880930a3/Library/Formula/fltk.rb
    brew install ode
    brew install bullet
    brew install suite-sparse

after make virtual environment,

    pip install --upgrade pip setuptools
    pip install numpy
    pip install PyOpenGL PyOpenGL-accelerate
    pip install cvxopt
    pip install pyode
    pip install openopt

setup pyfltk1.3.3 (google and download)
     in python/fltk\_wrap.cpp,
     add void in front of free\_color

     python setup.py build
     python setup.py install


for dart setup,
    brew install eigen assimp homebrew/science/libccd dartsim/dart/fcl boost open-scene-graph
    brew install homebrew/science/nlopt homebrew/science/ipopt tinyxml tinyxml2 ros/deps/urdfdom
	# brew install homebrew/science/flann  # it seems that it doesn't work.....

    git clone https://github.com/dartsim/dart
    cd dart;git checkout tags/v6.0.1;mkdir build;cd build;cmake ..
    make -j4
