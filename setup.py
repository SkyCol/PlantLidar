from setuptools import setup, find_packages

setup(
    name='PlantLidar',
    version='1.0.0',
    author='Your Name',
    author_email='s1729041183@gmail.com',
    description='A lidar package for Plant Phenotype researching',
    packages=find_packages(),
    install_requires=[
        'open3d',
        'numpy',
        'pandas',
        'CSF',
        'tqdm',
        'ctypes',
        'matplotlib',
        'scipy'
    ],
)
