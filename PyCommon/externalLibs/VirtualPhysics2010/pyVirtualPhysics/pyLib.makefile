UNAME := $(shell uname -s)
ifeq ($(UNAME), Darwin)
        PREPROCESSOR = -Xprocessor -fopenmp -D __APPLE_OMP__
        CPP_LIB_OMP_FLAG = -lomp -fopenmp
        CPP_COMPILER = clang++
        C_COMPILER = clang++
        LIB_FLAG = -bundle
else ifeq ($(UNAME), Linux)
        CPP_COMPILER = g++
        C_COMPILER = gcc
        PREPROCESSOR = -fopenmp
        COMP_FLAG = -fPIC
        LIB_FLAG = -shared
        CPP_LIB_OMP_FLAG = -lgomp -fopenmp
endif
NUMPY_INC_DIR := $(shell python -c "import numpy;print(numpy.get_include())")

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
