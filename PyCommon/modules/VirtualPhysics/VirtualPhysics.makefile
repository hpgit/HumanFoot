UNAME := $(shell uname -s)
PYVER := $(shell python -V | sed "s/Python \([0-9]*\).\([0-9]*\).\([0-9]*\)/\1.\2/")
ifeq ($(UNAME), Darwin)
	MACVER := $(shell sw_vers -productVersion | sed "s:.[[:digit:]]*.$$::g")
	FOLDER := lib.macosx-$(MACVER)-x86_64-$(PYVER)
	MAC_OMP := $(shell clang-omp++ --version 2>/dev/null)
ifdef MAC_OMP
	SETUPFILE := setup_mac_omp.py
else
	SETUPFILE := setup_mac.py
endif
endif
ifeq ($(UNAME), Linux)
	FOLDER := lib.linux-x86_64-$(PYVER)
	SETUPFILE := setup.py
endif

all:
	#python setup.py build ; cp build/lib.linux-x86_64-2.7/*.so ./
	python $(SETUPFILE) build $(SETUPARG); cp build/$(FOLDER)/*.so ./

clean:
	rm -rf build

