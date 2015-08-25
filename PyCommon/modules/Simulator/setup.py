from distutils.core import setup, Extension

module_csVpModel = Extension('csVpModel',
		include_dirs = ['../usr/include/'],
		extra_compile_args=['-fopenmp'],
		extra_link_args=['-lgomp'],
		libraries = ['boost_python', 'vpLib'],
		library_dirs = ['../usr/lib'],
		sources = ['csVpModel.cpp', 'VpUtil.cpp', 'myGeom.cpp'])
	
setup (name = 'csVpModel',
	version = '0.1',
	description = 'csVpModel',
	ext_modules = [module_csVpModel])

module_csVpWorld = Extension('csVpWorld',
		include_dirs = ['../usr/include/'],
		extra_compile_args=['-fopenmp'],
		extra_link_args=['-lgomp'],
		libraries = ['boost_python', 'vpLib'],
		library_dirs = ['../usr/lib'],
		sources = ['csVpWorld.cpp', 'VpUtil.cpp'])
	
setup (name = 'csVpWorld',
	version = '0.1',
	description = 'csVpWorld',
	ext_modules = [module_csVpWorld])
