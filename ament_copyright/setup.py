from setuptools import find_packages
from setuptools import setup

setup(
    name='ament_copyright',
    version='0.6.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    package_data={'': [
        'template/*',
    ]},
    zip_safe=False,
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
    long_description="""\
The ability to check sources file for copyright and license information.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ament_copyright.copyright_name': [
            'osrf = ament_copyright.copyright_names:osrf',
        ],
        'ament_copyright.license': [
            'apache2 = ament_copyright.licenses:apache2',
            'bsd2 = ament_copyright.licenses:bsd2',
            'mit = ament_copyright.licenses:mit',
        ],
        'console_scripts': [
            'ament_copyright = ament_copyright.main:main',
        ],
    },
)
