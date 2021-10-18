from setuptools import find_packages
from setuptools import setup

package_name = 'ament_cobra'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Michael Jeronimo',
    author_email='michael.jeronimo@openrobotics.org',
    maintainer='Michael Jeronimo',
    maintainer_email='michael.jeronimo@openrobotics.org',
    url='https://github.com/ament/ament_lint',
    download_url='https://github.com/ament/ament_lint/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Static code analysis using Cobra.',
    long_description="""\
The ability to perform static code analysis on source code using the
Cobra static analyzer and generate xUnit test result files.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ament_cobra = ament_cobra.main:main',
        ],
    },
)
