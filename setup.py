from setuptools import setup
import os

with open("description.md", "r") as fh:
    long_description = fh.read()

version_file = open('VERSION')
version = version_file.read().strip()

setup(name='bpybullet',
      version=version,
      description='library to link the blender python api (bpy) with pybullet.',
      long_description=long_description,
      long_description_content_type="text/markdown",
      author='Tobias Lemke',
      url="",
      packages=['bpybullet'],
      install_requires=[],
      classifiers=[])

