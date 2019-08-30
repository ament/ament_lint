from setuptools import find_packages
from setuptools import setup

package_name = 'ament_pep257'

setup(
    name=package_name,
    version='0.7.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    package_data={'': [
        'configuration/ament_pep257.ini',
    ]},
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
    description='Check Python code style using pep257.',
    long_description="""\
The ability to check code against the docstring conventions in PEP 257
and generate xUnit test result files.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ament_pep257 = ament_pep257.main:main',
        ],
        'pytest11': [
            'ament_pep257 = ament_pep257.pytest_marker',
        ],
    },
)
