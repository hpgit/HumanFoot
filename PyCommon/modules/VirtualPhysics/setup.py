from distutils.core import setup, Extension

def moduleSetup(moduleName):
	moduleToSetup = Extension(moduleName,
		include_dirs = ['../usr/include/'],
		extra_compile_args=['-fopenmp'],
		extra_link_args=['-lgomp'],
		libraries = ['boost_python', 'vpLib'],
		library_dirs = ['../usr/lib'],
		sources = ['pyV'+moduleName[1:]+'.cpp'])
	
	setup (name = moduleName,
		version = '0.1',
		description = moduleName,
		ext_modules = [moduleToSetup])

moduleSetup('vpWorld')
moduleSetup('vpBody')
