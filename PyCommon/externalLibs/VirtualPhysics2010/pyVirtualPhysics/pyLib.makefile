UNAME := $(shell uname -s)
ifeq ($(UNAME), Darwin)
	MAC_OMP := $(shell clang-omp++ --version 2>/dev/null)
ifdef MAC_OMP
	CPP_COMPILER = clang-omp++
	C_COMPILER = clang-omp++
	PREPROCESSOR = -D __APPLE_OMP__ -fopenmp
else
	CPP_COMPILER = clang++
	C_COMPILER = clang++
endif
endif

ifeq ($(UNAME), Linux)
	CPP_COMPILER = g++
	C_COMPILER = gcc
	PREPROCESSOR = -fopenmp
endif
NUMPY_INC_DIR := $(shell python -c "import numpy;print numpy.get_include()")

.PHONY: Release
Release:
	ln -s ../usr/include/VP ./
	swig -python -c++ pyVirtualPhysics.i
	$(CPP_COMPILER) -I../usr/include -I$(NUMPY_INC_DIR) `python-config --cflags` $(PREPROCESSOR) -c ../vpLib/*.cpp pyVirtualPhysics_wrap.cxx
	$(CPP_COMPILER) -bundle `python-config --ldflags` -liomp5 *.o -o _pyVirtualPhysics.so
	rm VP


.PHONY: clean
clean:
	rm -f *.o
	rm -f *.so
	rm -f *.py
	rm -f *.pyc
	rm -f *.cxx
