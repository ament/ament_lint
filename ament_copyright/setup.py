from setuptools import find_packages
from setuptools import setup

setup(
    name='ament_copyright',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    author='Dirk Thomas',
    author_email='dthomas@osrfoundation.org',
    maintainer='Dirk Thomas',
    maintainer_email='dthomas@osrfoundation.org',
    url='https://github.com/ament/ament_lint',
    download_url='https://github.com/ament/ament_lint/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Check source files for copyright reference.',
    long_description='''\
The ability to check every source file contains copyright reference
in the ament buildsystem.''',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'ament_copyright = ament_copyright:main',
        ],
        'ament_copyright.names': [
            'osrf = ament_copyright.names:osrf',
        ],
    },
)
