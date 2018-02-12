from distutils.core import setup, Extension
import sys
py_major_ver = sys.version_info[0]

boost_lib = 'boost_python'

if py_major_ver == 3:
    boost_lib = boost_lib + '3'

def moduleSetup(moduleName):
    moduleToSetup = Extension(moduleName,
        include_dirs = ['../usr/include/'],
        extra_compile_args=['-fopenmp'],
        extra_link_args=['-lgomp'],
        libraries = [boost_lib, 'vpLib'],
        library_dirs = ['../usr/lib'],
        sources = ['pyV'+moduleName[1:]+'.cpp'])

    setup (name = moduleName,
        version = '0.1',
        description = moduleName,
        ext_modules = [moduleToSetup])

moduleSetup('vpWorld')
moduleSetup('vpBody')
