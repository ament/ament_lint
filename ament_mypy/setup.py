from setuptools import find_packages
from setuptools import setup

setup(
    name='ament_mypy',
    version='0.7.3',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    package_data={'': [
        'configuration/ament_mypy.ini',
    ]},
    zip_safe=False,
    author='Ted Kern',
    author_email='ted.kern@canonical.com',
    maintainer='Ted Kern',
    maintainer_email='ted.kern@canonical.com',
    url='https://github.com/ament/ament_lint',
    download_url='https://github.com/ament/ament_lint/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Check Python static typing using mypy.',
    long_description="""\
The ability to check code for user specified static typing with mypy.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ament_mypy = ament_mypy.main:main',
        ],
    },
)
