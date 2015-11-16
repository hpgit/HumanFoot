MODULE_INC_DIR := PyCommon/modules/usr/include
MODULE_LIB_DIR := PyCommon/modules/usr/lib
VP_DIR := PyCommon/externalLibs/VirtualPhysics2010
GEAR_DIR := PyCommon/externalLibs/gear
PYTHON_FILE := main_MomentumProject_SimpleLCP.py
#PYTHON_FILE := ompTest.py
PYTHON_FILE := main_Test.py
#PYTHON_FILE := lcpTest2.py

all:
	cd PyCommon/modules;make

install_dependencies:
	sudo apt-get install python-dev python-opengl freeglut3-dev pypy g++ python-fltk python-numpy python-pyode libgle3 python-pip liblapack-dev libblas-dev libboost-python-dev
	pip install cvxopt --user
	[ -e $(MODULE_INC_DIR) ] || mkdir -p $(MODULE_INC_DIR)
	cd $(MODULE_INC_DIR);[ -e VP ] || ln -s ../../../../$(VP_DIR)/usr/include/VP ./ ;[ -e gear ] || ln -s ../../../../$(GEAR_DIR)/include ./gear
	cd $(VP_DIR);make;cd ../../modules/usr;[ -e lib ] || mkdir lib;cd lib;[ ! -e libvpLib.a ] || rm libvpLib.a
	cp $(VP_DIR)/usr/lib/Win32/gccRelease/libvpLib.a $(MODULE_LIB_DIR)/


install_dependencies_mac:
	[ -e $(MODULE_INC_DIR) ] || mkdir -p $(MODULE_INC_DIR)
	cd $(MODULE_INC_DIR);[ -e VP ] || ln -s ../../../../$(VP_DIR)/usr/include/VP ./ ;[ -e gear ] || ln -s ../../../../$(GEAR_DIR)/include ./gear
	cd $(VP_DIR);make;cd ../../modules/usr;[ -e lib ] || mkdir lib;cd lib;[ ! -e libvpLib.a ] || rm libvpLib.a
	cp $(VP_DIR)/usr/lib/Win32/gccRelease/libvpLib.a $(MODULE_LIB_DIR)/

run:
	cd MomentumProject;python $(PYTHON_FILE)

clean:
	cd PyCommon/modules;make clean
