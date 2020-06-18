from setuptools import setup, find_packages

# read the README file
from os import path

this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    readme = f.read()

with open(path.join(this_directory, 'requirements.txt'), encoding='utf-8') as f:
    required = f.read().splitlines()

setup(
    name='commonroad-route-planner',
    version='0.9.0',
    description='route planner for the commonroad framework',
    keywords='autonomous automated vehicles driving motion planning',
    url='https://gitlab.lrz.de/cps/commonroad-route-planner',
    author='Daniel Tar',
    author_email='daniel.tar@tum.de',
    license='GNU General Public License v3.0',
    packages=find_packages(),
    install_requires=required,
    extras_require={},
    long_description_content_type='text/markdown',
    long_description=readme,
    classifiers=[
        "Programming Language :: Python :: 3.7",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS",
    ],
    data_files=[('.', ['LICENSE.txt'])],
    include_package_data=True,
)
