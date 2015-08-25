all:
	cd PyCommon/modules;make

install_dependencies:
	sudo apt-get install python-dev python-opengl freeglut3-dev pypy g++ python-fltk python-numpy python-pyode libgle3 python-pip liblapack-dev libblas-dev libboost-python-dev
	pip install cvxopt --user
	cd PyCommon/externalLibs/VirtualPhysics2010;make;mkdir ../../modules/usr/lib;cp usr/lib/Win32/gccRelease/libvpLib.a ../../modules/usr/lib/

run:
	cd MomentumProject;python main_MomentumProject.py
