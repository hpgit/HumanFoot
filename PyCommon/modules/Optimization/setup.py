from distutils.core import setup, Extension

module_csLCPLemkeSolver = Extension('csLCPLemkeSolver',
		include_dirs = ['../usr/include/', '/usr/local/include/bullet/'],
		extra_compile_args=['-fopenmp'],
		extra_link_args=['-lgomp'],
		libraries = ['boost_python', 'LinearMath'],
		library_dirs = ['../usr/lib'],
		sources = ['csLCPLemkeSolver.cpp'])
	
setup (name = 'csLCPLemkeSolver',
	version = '0.1',
	description = 'csLCPLemkeSolver',
	ext_modules = [module_csLCPLemkeSolver])

module_csLCPDantzigSolver = Extension('csLCPDantzigSolver',
		include_dirs = ['../usr/include/', '/usr/local/include/bullet', '/usr/local/include/libiomp'],
		extra_compile_args=['-fopenmp'],
		extra_link_args=['-lgomp'],
		libraries = ['boost_python', 'LinearMath'],
		library_dirs = ['../usr/lib'],
		sources = ['csLCPDantzigSolver.cpp','btDantzigSolver.cpp', 'btDantzigAlgorithm.cpp'])
	
setup (name = 'csLCPDantzigSolver',
	version = '0.1',
	description = 'csLCPDantzigSolver',
	ext_modules = [module_csLCPDantzigSolver])