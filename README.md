# momentum
1. first, `ctags -R .`

# vundle (old version)
	git clone https://github.com/gmarik/vundle.git ~/.vim/bundle/vundle

## How to build and run
#### In linux,
1. setup boost
	
		$ ./bootstrap.sh
		$ sudo ./b2 install
	
2. sudo apt-get install
python-dev
python-opengl
freeglut3-dev
pypy
g++
python-fltk
python-numpy
python-pyode
libgle3

3. build VirtualPhysics2010 (remove -fPIC in vpLib.makefile) & copy libvpLib.a to PyCommon/modules/usr/lib/
4. make in PyCommon/modules (if there is a copy error, edit setup.py in Renderer and Simulator)
5. Run main_MomentumProject.py in MomentumProject/

#### In MAC OS X,

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

after make virtual environment,

    pip install --upgrade pip setuptools
    pip install numpy
    pip install PyOpenGL PyOpenGL-accelerate
    pip install cvxopt
    pip install pyode
    pip install openopt

setup pyfltk1.3.3 (google and download)
     in python/fltk_wrap.cpp,
     add void in front of free_color

     python setup.py build
     python setup.py install








