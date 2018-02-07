from distutils.core import setup, Extension
import sys


class setupmodule:
    def __init__(self, name='noName'):
        self.name = name
        self.include_dirs = ['../usr/include/']
        self.extra_compile_args = []
        self.libraries = []
        self.library_dirs = ['../usr/lib']
        self.extra_link_args = []
        self.sources = [name+'.cpp']
        self.depends = [name+'.h']


isMAC = False
isOMP = True
ompLib = 'gomp'

if '--with-mac-omp' in sys.argv:
    isMAC = True
    ompLib = 'iomp5'
    idx = sys.argv.index('--with-mac-omp')
    sys.argv.pop(idx)
elif '--with-mac' in sys.argv:
    isMAC = True
    isOMP = False
    idx = sys.argv.index('--with-mac')
    sys.argv.pop(idx)

modules = []

m = setupmodule('csMath')
m.include_dirs = ['../usr/include/']
m.libraries = ['boost_python', 'boost_numpy', 'vpLib', ompLib]
if isMAC and isOMP:
    m.extra_compile_args = ['-fopenmp', '-D __APPLE_OMP__']
elif isOMP:
    m.extra_compile_args = ['-fopenmp']
else:
    m.libraries.pop()
m.library_dirs = ['../usr/lib']
m.sources.append('EulerAngles.cpp')
m.depends.extend(['stdafx.h', 'EulerAngles.h', 'QuatTypes.h'])
modules.append(m)


for m in modules:
    ext_module = Extension(m.name,
            include_dirs=m.include_dirs,
            extra_compile_args=m.extra_compile_args,
            extra_link_args=m.extra_link_args,
            libraries=m.libraries,
            library_dirs=m.library_dirs,
            sources=m.sources,
            depends=m.depends)

    setup(name=m.name, ext_modules=[ext_module])
    
'''
setup (name = 'csVpModel',
    version = '0.1',
    description = 'csVpModel',
    ext_modules = [module_csVpModel])

setup (name = 'csVpWorld',
    version = '0.1',
    description = 'csVpWorld',
    ext_modules = [module_csVpWorld])
'''

