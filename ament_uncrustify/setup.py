from setuptools import find_packages
from setuptools import setup

package_name = 'ament_uncrustify'

setup(
    name=package_name,
    version='0.7.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    package_data={'': [
        'configuration/ament_code_style.cfg',
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
    description='Check code style using uncrustify.',
    long_description="""\
The ability to check code against style conventions using uncrustify
and generate xUnit test result files.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ament_uncrustify = ament_uncrustify.main:main',
        ],
    },
)
