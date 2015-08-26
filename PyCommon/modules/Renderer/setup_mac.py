from distutils.core import setup, Extension

module_csVpRenderer = Extension('csVpRenderer',
		include_dirs = ['../usr/include/'],
#extra_compile_args=['-fopenmp'],
#extra_link_args=['-lgomp'],
		libraries = ['boost_python','vpLib'],
		library_dirs = ['../usr/lib'],
		sources = ['csVpRenderer.cpp'])
	
setup (name = 'csVpRenderer',
	version = '0.1',
	description = 'csVpRenderer',
	ext_modules = [module_csVpRenderer])
