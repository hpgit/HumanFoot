MODULE_INC_DIR := PyCommon/modules/usr/include
MODULE_LIB_DIR := PyCommon/modules/usr/lib
VP_DIR := PyCommon/externalLibs/VirtualPhysics2010
GEAR_DIR := PyCommon/externalLibs/gear
qpOASES_DIR := PyCommon/externalLibs/qpOASES
PYDART2_DIR := PyCommon/externalLibs/pydart2
#PYTHON_FILE := main_MomentumProject_SimpleLCP.py
#PYTHON_FILE := ompTest.py
#PYTHON_FILE := main_Test.py
#PYTHON_FILE := lcpTest2.py
PYTHON_FILE := run.py

all:
	cd PyCommon/modules;make

install_dependencies:
	sudo apt install build-essential cmake
	sudo apt install swig python-dev python-opengl freeglut3-dev g++ python-fltk python-numpy python-pyode libgle3 python-pip liblapack-dev libblas-dev libboost-python-dev python-scipy
	pip install cvxopt future six pillow colorama
	[ -e $(MODULE_INC_DIR) ] || mkdir -p $(MODULE_INC_DIR)
	cd $(MODULE_INC_DIR);[ -e VP ] || ln -s ../../../../$(VP_DIR)/usr/include/VP ./ ;[ -e gear ] || ln -s ../../../../$(GEAR_DIR)/include ./gear;[ -e qpOASES ] || ln -s ../../../../$(qpOASES_DIR)/include ./qpOASES
	cd PyCommon/modules/usr;[ -e lib ] || mkdir lib;cd lib;[ ! -e libvpLib.a ] || rm libvpLib.a
	cd $(VP_DIR);[ -e build ] || mkdir build;cd build;cmake ..;make -j2
	cp $(VP_DIR)/build/libvpLib.a $(MODULE_LIB_DIR)/
	cd $(qpOASES_DIR);[ -e bin ] || mkdir -p bin;make
	cp $(qpOASES_DIR)/bin/libqpOASES.a $(MODULE_LIB_DIR)/
	# cd $(PYDART2_DIR);make

install_dart_ubuntu:
	sudo apt install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex1.58-dev libboost-system1.58-dev libopenscenegraph-dev
	sudo apt install libnlopt-dev coinor-libipopt-dev libbullet-dev libflann-dev libtinyxml-dev libtinyxml2-dev liburdfdom-dev liburdfdom-headers-dev libxi-dev libxmu-dev freeglut3-dev doxygen
	cd PyCommon/externalLibs;git clone https://github.com/dartsim/dart;cd dart;git checkout tags/v6.1.2;mkdir build;cd build;cmake ..;make -j2;sudo make install

install_dependencies_mac:
	pip install -r requirements.txt
	[ -e $(MODULE_INC_DIR) ] || mkdir -p $(MODULE_INC_DIR)
	cd $(MODULE_INC_DIR);[ -e VP ] || ln -s ../../../../$(VP_DIR)/usr/include/VP ./ ;[ -e gear ] || ln -s ../../../../$(GEAR_DIR)/include ./gear;[ -e qpOASES ] || ln -s ../../../../$(qpOASES_DIR)/include ./qpOASES
	cd PyCommon/modules/usr;[ -e lib ] || mkdir lib;cd lib;[ ! -e libvpLib.a ] || rm libvpLib.a
	cd $(VP_DIR);[ -e build ] || mkdir build;cd build;cmake ..;make
	cp $(VP_DIR)/build/libvpLib.a $(MODULE_LIB_DIR)/
	cd $(qpOASES_DIR);[ -e bin ] || mkdir -p bin;make
	cp $(qpOASES_DIR)/bin/libqpOASES.a $(MODULE_LIB_DIR)/
	# cd $(PYDART2_DIR);make

run:
	cd MomentumProject;python -i $(PYTHON_FILE)

clean:
	cd PyCommon/modules;make clean
