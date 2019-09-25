from setuptools import find_packages
from setuptools import setup

package_name = 'ament_pclint'

setup(
    name=package_name,
    version='0.8.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    package_data={'': [
        'config/gcc/co-g++.h',
        'config/gcc/co-g++.lnt',
        'config/gcc/co-gcc.h',
        'config/gcc/co-gcc.lnt',
        'config/gcc/co-osx-g++.h',
        'config/gcc/co-osx-g++.lnt',
        'config/gcc/co-osx-gcc.h',
        'config/gcc/co-osx-gcc.lnt',
        'config/msvc/cl-include-path.lnt',
        'config/msvc/co-cl.lnt',
        'config/msvc/co-cl++.lnt',
        'config/msvc/co-cl.h',
        'config/au-misra3.lnt',
        'config/au-misra3-amd1.lnt',
        'config/au-misra-cpp.lnt',
        'config/c99.lnt',
        'config/c++.lnt',
        'config/deprecate.lnt',
        'config/env-xml.lnt'
    ]},
    zip_safe=False,
    author='Juan Pablo Samper',
    author_email='jp.samper@apex.ai',
    maintainer='Juan Pablo Samper',
    maintainer_email='jp.samper@apex.ai',
    url='https://github.com/ament/ament_lint',
    download_url='',
    keywords=['ament'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apex AI',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Static code analysis on C/C++ code using PCLint.',
    long_description="""\
The ability to perform static code analysis on C/C++ code using PCLint
and generate xUnit test result files.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    test_suite='test',
    entry_points={
        'console_scripts': [
            'ament_pclint = ament_pclint.main:main',
        ],
    },
)
