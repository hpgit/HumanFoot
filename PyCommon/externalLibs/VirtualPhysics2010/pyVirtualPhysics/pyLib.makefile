UNAME := $(shell uname -s)
ifeq ($(UNAME), Darwin)
	MAC_OMP := $(shell clang-omp++ --version 2>/dev/null)
ifdef MAC_OMP
        CPP_COMPILER = clang-omp++
        C_COMPILER = clang-omp++
        PREPROCESSOR = -D __APPLE_OMP__ -fopenmp
        CPP_LIB_OMP_FLAG = -liomp5
        LIB_FLAG = -bundle
else
        CPP_COMPILER = clang++
        C_COMPILER = clang++
        LIB_FLAG = -bundle
endif
endif

ifeq ($(UNAME), Linux)
        CPP_COMPILER = g++
        C_COMPILER = gcc
        PREPROCESSOR = -fopenmp
        COMP_FLAG = -fPIC
        LIB_FLAG = -shared
        CPP_LIB_OMP_FLAG = -lgomp -fopenmp
endif
NUMPY_INC_DIR := $(shell python -c "import numpy;print numpy.get_include()")

.PHONY: Release
Release:
        [ -e VP ] || ln -s ../usr/include/VP ./
        swig -python -c++ pyVirtualPhysics.i
        $(CPP_COMPILER) -I../usr/include -I$(NUMPY_INC_DIR) `python-config --cflags` $(COMP_FLAG) $(PREPROCESSOR) -c ../vpLib/*.cpp pyVirtualPhysics_wrap.cxx
        $(CPP_COMPILER) $(LIB_FLAG) `python-config --ldflags` $(CPP_LIB_OMP_FLAG) *.o  -o _pyVirtualPhysics.so
        cp pyVirtualPhysics.py ../../../modules/
        cp _pyVirtualPhysics.so ../../../modules/


.PHONY: clean
clean:
	rm -f *.o
	rm -f *.so
	rm -f *.py
	rm -f *.pyc
	rm -f *.cxx
