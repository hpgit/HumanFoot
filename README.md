# momentum
1. first, `ctags -R .`

# vundle (old version)
	git clone https://github.com/gmarik/vundle.git ~/.vim/bundle/vundle

## How to run
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
- python-opengl : pip install PyOpenGL PyOpenGL_accelerate
- 
