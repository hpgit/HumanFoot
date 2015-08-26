MODULE_INC_DIR := PyCommon/modules/usr/include
MODULE_LIB_DIR := PyCommon/modules/usr/lib
VP_DIR := PyCommon/externalLibs/VirtualPhysics2010
GEAR_DIR := PyCommon/externalLibs/gear

all:
	cd PyCommon/modules;make

install_dependencies:
	sudo apt-get install python-dev python-opengl freeglut3-dev pypy g++ python-fltk python-numpy python-pyode libgle3 python-pip liblapack-dev libblas-dev libboost-python-dev
	pip install cvxopt --user
	[ -d $(MODULE_INC_DIR) ] || mkdir -p $(MODULE_INC_DIR)
	cd $(MODULE_INC_DIR);[ -d VP ] || ln -s ../../../../$(VP_DIR)/usr/include/VP ./ ;[ -d gear ] || ln -s ../../../../$(GEAR_DIR)/include ./gear
	cd PyCommon/externalLibs/VirtualPhysics2010;make;mkdir ../../modules/usr/lib;cp usr/lib/Win32/gccRelease/libvpLib.a ../../modules/usr/lib/

install_dependencies_mac:
	mkdir -p PyCommon/modules/usr/include;cd PyCommon/modules/usr/include;ln -s ../../../externalLibs/VirtualPhysics2010/usr/include/VP;ln -s ../../../externalLibs/VirtualPhysics2010/usr/include/gear
	cd PyCommon/externalLibs/VirtualPhysics2010;make;mkdir ../../modules/usr/lib;cp usr/lib/Win32/gccRelease/libvpLib.a ../../modules/usr/lib/

run:
	cd MomentumProject;python main_MomentumProject.py

clean:
	cd PyCommon/modules;make clean
