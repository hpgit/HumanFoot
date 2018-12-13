MODULE_INC_DIR := PyCommon/modules/usr/include
MODULE_LIB_DIR := PyCommon/modules/usr/lib
qpOASES_DIR := PyCommon/externalLibs/qpOASES
#PYTHON_FILE := main_MomentumProject_SimpleLCP.py
#PYTHON_FILE := ompTest.py
#PYTHON_FILE := main_Test.py
#PYTHON_FILE := lcpTest2.py
PYTHON_FILE := run.py

all:
	mkdir build;cd build;cmake ..
	cmake --build build --target all -j

install_dependencies:
	sudo apt install build-essential cmake
	sudo apt install swig python-dev python-opengl freeglut3-dev g++ python-fltk python-numpy python-pyode libgle3 python-pip liblapack-dev libblas-dev libboost-python-dev python-scipy
	pip install cvxopt future six pillow colorama
	[ -e $(MODULE_INC_DIR) ] || mkdir -p $(MODULE_INC_DIR)
	cd $(MODULE_INC_DIR);[ -e qpOASES ] || ln -s ../../../../$(qpOASES_DIR)/include ./qpOASES
	cd PyCommon/modules/usr;[ -e lib ] || mkdir lib
	cd $(qpOASES_DIR);[ -e bin ] || mkdir -p bin;make
	cp $(qpOASES_DIR)/bin/libqpOASES.a $(MODULE_LIB_DIR)/

install_dart_ubuntu:
	sudo apt install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex1.58-dev libboost-system1.58-dev libopenscenegraph-dev
	sudo apt install libnlopt-dev coinor-libipopt-dev libbullet-dev libflann-dev libtinyxml-dev libtinyxml2-dev liburdfdom-dev liburdfdom-headers-dev libxi-dev libxmu-dev freeglut3-dev doxygen
	cd PyCommon/externalLibs;git clone https://github.com/dartsim/dart;cd dart;git checkout tags/v6.1.2;mkdir build;cd build;cmake ..;make -j2;sudo make install

install_dependencies_mac:
	pip install -r requirements.txt
	[ -e $(MODULE_INC_DIR) ] || mkdir -p $(MODULE_INC_DIR)
	cd $(MODULE_INC_DIR);[ -e qpOASES ] || ln -s ../../../../$(qpOASES_DIR)/include ./qpOASES
	cd PyCommon/modules/usr;[ -e lib ] || mkdir lib
	cd $(qpOASES_DIR);[ -e bin ] || mkdir -p bin;make
	cp $(qpOASES_DIR)/bin/libqpOASES.a $(MODULE_LIB_DIR)/

run:
	cd MomentumProject;python -i $(PYTHON_FILE)

clean:
	cmake --build build --target clean
