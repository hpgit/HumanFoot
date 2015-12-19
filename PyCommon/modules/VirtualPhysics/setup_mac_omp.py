from distutils.core import setup, Extension


def moduleSetup(moduleName):
    moduleToSetup = Extension(moduleName,
                        include_dirs=['../usr/include/', '/usr/local/include/libiomp'],
                        extra_compile_args=['-fopenmp', '-D __APPLE_OMP__'],
                        libraries=['boost_python', 'vpLib', 'iomp5'],
                        library_dirs=['../usr/lib'],
                        sources=['pyV'+moduleName[1:]+'.cpp'])

    setup(name=moduleName,
          version='0.1',
          description=moduleName,
          ext_modules=[moduleToSetup])

moduleSetup('vpWorld')
moduleSetup('vpBody')
moduleSetup('vpBJoint')
moduleSetup('vpMaterial')
moduleSetup('vpSystem')
moduleSetup('vpGeom')
