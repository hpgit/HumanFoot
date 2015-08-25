all:
	python setup.py build ; cp build/lib.linux-x86_64-2.7/*.so ./

clean:
	rm -rf build

