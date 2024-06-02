# -*- coding: utf-8 -*-
import os
import subprocess
from os import path

from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext

name = 'IsoOctree'
root_dir = path.abspath(os.path.join(path.dirname(__file__), '..'))
this_is_my_real_build_dir = os.path.join(root_dir, 'python', 'target')

with open(path.join(root_dir, 'VERSION.txt')) as f:
    version = f.read().strip()

# Get the long description from the README file
long_description = ''
#with open(path.join(here, 'long_description.rst'), encoding='utf-8') as f:
#    long_description = f.read()

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)

class JustUseMyExistingBuild(build_ext):
    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep
        os.makedirs(extdir, exist_ok=True) # Required for the spectacularAI_native hack to work

        here_distutils_would_like_it_to_be = extdir
        subprocess.check_call('cp -r ' + name + '*.* ' + here_distutils_would_like_it_to_be, shell=True, cwd=this_is_my_real_build_dir)

setup(
    name=name,
    version=version,
    author="Spectacular AI Ltd",
    url='https://github.com/SpectacularAI/IsoOctree',
    author_email="apps@spectacularai.com",
    long_description=long_description,
    license='BSD-3-Clause', # TODO
    keywords='mesh',
    python_requires='>=3',
    install_requires=['numpy'],
    packages=find_packages(), # Find Python packages, including subpackages!
    ext_modules=[CMakeExtension(name)],
    cmdclass={"build_ext": JustUseMyExistingBuild},
    zip_safe=False
)