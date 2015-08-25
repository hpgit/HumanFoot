all:
	cd PyCommon/modules;make

install_dependencies:
	sudo apt-get install python-dev python-opengl freeglut3-dev pypy g++ python-fltk python-numpy python-pyode libgle3 python-pip liblapack-dev libblas-dev libboost-python-dev
	pip install cvxopt --user
	mkdir -p PyCommon/modules/usr/include;ln -s PyCommon/externalLibs/VirtualPhysics2010/usr/include/VP PyCommon/modules/usr/include/VP;ln -s PyCommon/externalLibs/VirtualPhysics2010/usr/include/gear PyCommon/modules/usr/include/gear
	cd PyCommon/externalLibs/VirtualPhysics2010;make;mkdir ../../modules/usr/lib;cp usr/lib/Win32/gccRelease/libvpLib.a ../../modules/usr/lib/

install_dependencies_mac:
	mkdir -p PyCommon/modules/usr/include;cd PyCommon/modules/usr/include;ln -s ../../../externalLibs/VirtualPhysics2010/usr/include/VP;ln -s ../../../externalLibs/VirtualPhysics2010/usr/include/gear

run:
	cd MomentumProject;python main_MomentumProject.py
