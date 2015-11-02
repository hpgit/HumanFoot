from distutils.core import setup, Extension

module_csLCPLemkeSolver = Extension('csLCPLemkeSolver',
		include_dirs = ['../usr/include/', '/usr/local/include/bullet/', '/usr/local/include/libiomp'],
		extra_compile_args=['-fopenmp', '-D __APPLE_OMP__'],
		libraries = ['boost_python', 'LinearMath', 'BulletDynamics', 'iomp5'],
		library_dirs = ['../usr/lib'],
		sources = ['csLCPLemkeSolver.cpp'])
	
setup (name = 'csLCPLemkeSolver',
	version = '0.1',
	description = 'csLCPLemkeSolver',
	ext_modules = [module_csLCPLemkeSolver])
