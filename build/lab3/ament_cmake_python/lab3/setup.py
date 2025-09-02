from setuptools import find_packages
from setuptools import setup

setup(
    name='lab3',
    version='0.0.0',
    packages=find_packages(
        include=('lab3', 'lab3.*')),
)
