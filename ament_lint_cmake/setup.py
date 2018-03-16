from setuptools import find_packages
from setuptools import setup

setup(
    name='ament_lint_cmake',
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
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
    description='Check CMake code style using cmakelint.',
    long_description="""\
The ability to lint CMake code using cmakelint and generate xUnit test
result files.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ament_lint_cmake = ament_lint_cmake.main:main',
        ],
    },
)
