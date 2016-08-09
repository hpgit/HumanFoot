UNAME := $(shell uname -s)
ifeq ($(UNAME), Darwin)
	MACVER := $(shell sw_vers -productVersion | sed "s:.[[:digit:]]*.$$::g")
	FOLDER := lib.macosx-$(MACVER)-x86_64-2.7
	MAC_OMP := $(shell clang-omp++ --version 2>/dev/null)
ifdef MAC_OMP
	SETUPFILE := setup_mac_omp.py
	SETUPARG := --with-mac-omp
else
	SETUPFILE := setup_mac.py
	SETUPARG := --with-mac
endif
endif
ifeq ($(UNAME), Linux)
	FOLDER := lib.linux-x86_64-2.7
	SETUPFILE := setup.py
endif
SETUPFILE := setup.py

all:
	#python setup.py build ; cp build/lib.linux-x86_64-2.7/*.so ./
	python $(SETUPFILE) build $(SETUPARG) ; cp build/$(FOLDER)/*.so ./

clean:
	cd build;rm -rf *

