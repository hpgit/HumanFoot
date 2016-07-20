all:
	[ ! -e dist ] || rm -rf dist
	cd build;make;make install
	python setup.py build;python setup.py install

first:
	[ -e build ] || mkdir build
	cd build;cmake ..
