# all:
# 	[ ! -e dist ] || rm -rf dist
#	[ ! -e build ] || rm -rf build;mkdir build
#	[ ! -e pydart/_pydart_api.so ] || rm pydart/_pydart_api.so
#	[ ! -e pydart/pydart_apiPYTHON_wrap.cxx ] || rm pydart/pydart_apiPYTHON_wrap.cxx
#	cd build;cmake ..;make;make install
#	python setup.py build;python setup.py install

UNAME := $(shell uname -s)
PYMAJORVER := $(shell python -V 2>&1 | sed "s/Python \([0-9]*\).\([0-9]*\).\([0-9]*\)/\1/")
ifeq ($(UNAME), Darwin)
    CMAKEFILE := CMakeLists_mac_py$(PYMAJORVER).txt
else ifeq ($(UNAME), Linux)
    CMAKEFILE := CMakeLists_linux_py$(PYMAJORVER).txt
endif

all:
	rm -rf build;mkdir build
	rm -rf pydart2/_pydart2_api.so
	rm -rf pydart2/pydart2_api.py
	rm -rf pydart2/pydart2_apiPYTHON_wrap.cxx
	rm -rf CMakeLists.txt
	ln -s $(CMAKEFILE) CMakeLists.txt
	cd build;cmake ..;make;make install
	python setup_cmake.py build
	cp -R build/lib*/pydart2 ../../modules/
	cp -R build/_pydart2_api.so ../../modules/pydart2/

pyonly:
	python setup_cmake.py build
	cp -R build/lib/pydart2 ../../modules/
	

clean:
	rm -rf build
	rm -rf pydart2/_pydart2_api.so
	rm -rf pydart2/pydart2_apiPYTHON_wrap.cxx
	rm -rf pydart2/pydart2_api.py
