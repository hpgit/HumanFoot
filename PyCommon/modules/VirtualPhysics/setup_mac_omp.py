from distutils.core import setup, Extension

module_vpWorld = Extension('vpWorld',
                           include_dirs=['../usr/include/', '/usr/local/include/libiomp'],
                           extra_compile_args=['-fopenmp', '-D __APPLE_OMP__'],
                           libraries=['boost_python', 'vpLib', 'iomp5'],
                           library_dirs=['../usr/lib'],
                           sources=['pyVpWorld.cpp'])

setup(name='vpWorld',
      version='0.1',
      description='vpWorld',
      ext_modules=[module_vpWorld])

module_vpBody = Extension('vpBody',
                           include_dirs=['../usr/include/', '/usr/local/include/libiomp'],
                           extra_compile_args=['-fopenmp', '-D __APPLE_OMP__'],
                           libraries=['boost_python', 'vpLib', 'iomp5'],
                           library_dirs=['../usr/lib'],
                           sources=['pyVpBody.cpp'])

setup(name='vpBody',
      version='0.1',
      description='vpBody',
      ext_modules=[module_vpBody])
