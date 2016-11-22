# all:
# 	[ ! -e dist ] || rm -rf dist
#	[ ! -e build ] || rm -rf build;mkdir build
#	[ ! -e pydart/_pydart_api.so ] || rm pydart/_pydart_api.so
#	[ ! -e pydart/pydart_apiPYTHON_wrap.cxx ] || rm pydart/pydart_apiPYTHON_wrap.cxx
#	cd build;cmake ..;make;make install
#	python setup.py build;python setup.py install

all:
	rm -rf build;mkdir build
	rm -rf pydart2/_pydart2_api.so
	rm -rf pydart2/pydart2_api.py
	rm -rf pydart2/pydart2_apiPYTHON_wrap.cxx
	cd build;cmake ..;make;make install
	python setup_cmake.py build
	cp -R build/lib/pydart2 ../../modules/
	cp -R build/_pydart2_api.so ../../modules/pydart2/

clean:
	rm -rf build
	rm -rf pydart2/_pydart2_api.so
	rm -rf pydart2/pydart2_apiPYTHON_wrap.cxx
	rm -rf pydart2/pydart2_api.py
