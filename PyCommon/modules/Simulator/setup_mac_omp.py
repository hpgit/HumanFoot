from distutils.core import setup, Extension

module_csVpModel = Extension('csVpModel',
		include_dirs = ['../usr/include/', '/usr/local/include/libiomp'],
		extra_compile_args=['-fopenmp', '-D __APPLE_OMP__'],
		libraries = ['boost_python', 'vpLib', 'iomp5'],
		library_dirs = ['../usr/lib'],
		sources = ['csVpModel.cpp', 'myGeom.cpp'])
	
setup (name = 'csVpModel',
	version = '0.1',
	description = 'csVpModel',
	ext_modules = [module_csVpModel])

module_csVpWorld = Extension('csVpWorld',
		include_dirs = ['../usr/include/', '/usr/local/include/libiomp'],
		extra_compile_args=['-fopenmp', '-D __APPLE_OMP__'],
		libraries = ['boost_python', 'vpLib', 'iomp5'],
		library_dirs = ['../usr/lib'],
		sources = ['csVpWorld.cpp'])
	
setup (name = 'csVpWorld',
	version = '0.1',
	description = 'csVpWorld',
	ext_modules = [module_csVpWorld])
