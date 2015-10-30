UNAME := $(shell uname -s)
ifeq ($(UNAME), Darwin)
	MACVER := $(shell sw_vers -productVersion | sed "s:.[[:digit:]]*.$$::g")
	FOLDER := lib.macosx-$(MACVER)-x86_64-2.7
	SETUPFILE := setup_mac.py
endif
ifeq ($(UNAME), Linux)
	FOLDER := lib.linux-x86_64-2.7
	SETUPFILE := setup.py
endif

all:
	#python setup.py build ; cp build/lib.linux-x86_64-2.7/*.so ./
	python $(SETUPFILE) build ; cp build/$(FOLDER)/*.so ./

clean:
	rm -rf build

