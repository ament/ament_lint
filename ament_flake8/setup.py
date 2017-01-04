from setuptools import find_packages
from setuptools import setup

setup(
    name='ament_flake8',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    package_data={'': [
        'configuration/ament_flake8.ini',
    ]},
    author='D. Hood',
    author_email='dhood@osrfoundation.org',
    maintainer='D. Hood',
    maintainer_email='dhood@osrfoundation.org',
    url='https://github.com/ament/ament_lint',
    download_url='https://github.com/ament/ament_lint/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Check Python code style using flake8.',
    long_description='''\
The ability to check code for syntax and style conventions with flake8.''',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'ament_flake8 = ament_flake8.main:main',
        ],
    },
)
