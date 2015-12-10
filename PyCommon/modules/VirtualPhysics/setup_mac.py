from distutils.core import setup, Extension

module_vpWorld = Extension('vpWorld',
						   include_dirs = ['../usr/include/'],
						   #extra_compile_args=['-fopenmp'],
						   #		extra_link_args=['-lgomp'],
						   libraries = ['boost_python', 'vpLib'],
						   library_dirs = ['../usr/lib'],
						   sources = ['pyVpWorld.cpp'])

setup (name = 'vpWorld',
	   version = '0.1',
	   description = 'vpWorld',
	   ext_modules = [module_vpWorld])

module_vpBody = Extension('vpBody',
		include_dirs = ['../usr/include/'],
#extra_compile_args=['-fopenmp'],
#		extra_link_args=['-lgomp'],
		libraries = ['boost_python', 'vpLib'],
		library_dirs = ['../usr/lib'],
		sources = ['pyVpBody.cpp'])

setup (name = 'vpBody',
	version = '0.1',
	description = 'vpBody',
	ext_modules = [module_vpBody])
