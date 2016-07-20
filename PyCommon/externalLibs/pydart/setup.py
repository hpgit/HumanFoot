from setuptools import setup,find_packages

setup(name='pydart',
      version='0.01',
      description='Python Interface for DART Simulator',
      url='https://github.com/sehoonha/pydart',
      author='Sehoon Ha',
      author_email='sehoon.ha@gmail.com',
      license='BSD',
      packages=find_packages(),
      package_data={'': ['*.so']},
      zip_safe=False)
