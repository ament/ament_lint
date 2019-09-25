from setuptools import find_packages
from setuptools import setup

package_name = 'ament_pyflakes'

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
    description='Check Python code style using pyflakes.',
    long_description="""\
The ability to check code using pyflakes and generate xUnit test
result files.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ament_pyflakes = ament_pyflakes.main:main',
        ],
    },
)
