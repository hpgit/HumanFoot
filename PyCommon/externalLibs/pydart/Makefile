all:
	[ ! -e dist ] || rm -rf dist
	[ ! -e build ] || rm -rf build;mkdir build
	[ ! -e pydart/_pydart_api.so ] || rm pydart/_pydart_api.so
	[ ! -e pydart/pydart_apiPYTHON_wrap.cxx ] || rm pydart/pydart_apiPYTHON_wrap.cxx
	cd build;cmake ..;make;make install
	python setup.py build;python setup.py install
