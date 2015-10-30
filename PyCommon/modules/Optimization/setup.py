from distutils.core import setup, Extension

module_csVpRenderer = Extension('csLCPLemkeSolver',
		include_dirs = ['../usr/include/', '/usr/local/include/bullet/'],
#extra_compile_args=['-fopenmp'],
#extra_link_args=['-lgomp'],
		libraries = ['boost_python', 'LinearMath', 'BulletDynamics'],
		library_dirs = ['../usr/lib'],
		sources = ['csLCPLemkeSolver.cpp'])
	
setup (name = 'csLCPLemkeSolver',
	version = '0.1',
	description = 'csLCPLemkeSolver',
	ext_modules = [module_csLCPLemkeSolver])
