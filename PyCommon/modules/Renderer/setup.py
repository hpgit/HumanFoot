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

m = setupmodule('csVpRenderer')
m.libraries = ['boost_python', 'vpLib', ompLib]
if isMAC and isOMP:
    m.extra_compile_args = ['-fopenmp', '-D __APPLE_OMP__']
elif isOMP:
    m.extra_compile_args = ['-fopenmp']
else:
    m.libraries.pop()
modules.append(m)

m = setupmodule('csIMSRenderer')
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
module_csVpRenderer = None
if '--with-mac-omp' in sys.argv:
    module_csVpRenderer = Extension('csVpRenderer',
            include_dirs = ['../usr/include/', '/usr/local/include/libiomp'],
            extra_compile_args=['-fopenmp', '-D __APPLE_OMP__'],
            libraries = ['boost_python', 'vpLib', 'iomp5'],
            library_dirs = ['../usr/lib'],
            sources = ['csVpRenderer.cpp'])
    idx = sys.argv.index('--with-mac-omp')
    sys.argv.pop(idx)
elif '--with-mac' in sys.argv:
    module_csVpRenderer = Extension('csVpRenderer',
            include_dirs = ['../usr/include/'],
            libraries = ['boost_python', 'vpLib'],
            library_dirs = ['../usr/lib'],
            sources = ['csVpRenderer.cpp'])
    idx = sys.argv.index('--with-mac')
    sys.argv.pop(idx)
else:
    module_csVpRenderer = Extension('csVpRenderer',
            include_dirs = ['../usr/include/'],
            extra_compile_args=['-fopenmp'],
            extra_link_args=['-lgomp'],
            libraries = ['boost_python', 'vpLib'],
            library_dirs = ['../usr/lib'],
            sources = ['csVpRenderer.cpp'])

setup (name = 'csVpRenderer',
    version = '0.1',
    description = 'csVpRenderer',
    ext_modules = [module_csVpRenderer])
'''