UNAME := $(shell uname -s)
PYVER := $(shell python -V 2>&1 | sed "s/Python \([0-9]*\).\([0-9]*\).\([0-9]*\)/\1.\2/")

ifeq ($(UNAME), Darwin)
	MACVER := $(shell sw_vers -productVersion | sed "s:.[[:digit:]]*.$$::g")
	FOLDER := lib.macosx-$(MACVER)-x86_64-$(PYVER)
	SETUPARG := --with-mac-omp
else ifeq ($(UNAME), Linux)
	FOLDER := lib.linux-x86_64-$(PYVER)
endif
SETUPFILE := setup.py

all:
	python $(SETUPFILE) build $(SETUPARG); cp build/$(FOLDER)/*.so ./

clean:
	rm -rf build

