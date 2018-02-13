from distutils.core import setup, Extension
import sys

module_csVpRenderer = Extension('csVpRenderer',
		include_dirs = ['../usr/include/', '/usr/local/include/libiomp'],
		extra_compile_args=['-fopenmp', '-D __APPLE_OMP__'],
		libraries = ['boost_python', 'vpLib', 'iomp5'],
		library_dirs = ['../usr/lib'],
		sources = ['csVpRenderer.cpp'])
	
setup (name = 'csVpRenderer',
	version = '0.1',
	description = 'csVpRenderer',
	ext_modules = [module_csVpRenderer])

