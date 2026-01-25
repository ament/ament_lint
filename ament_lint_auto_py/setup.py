from setuptools import find_packages
from setuptools import setup

package_name = 'ament_lint_auto_py'

setup(
    name=package_name,
    version='0.20.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    package_data={'': [
        'py.typed'
    ]},
    zip_safe=False,
    author='Ted Kern',
    author_email='ted.kern@canonical.com',
    maintainer='Michael Jeronimo',
    maintainer_email='michael.jeronimo@openrobotics.org',
    url='https://github.com/ament/ament_lint',
    download_url='https://github.com/ament/ament_lint/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Run ament linters',
    long_description='The auto-magic functions for ease to use of the ament linters in Python.',
    license='Apache License, Version 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'pytest11': [
            'ament_lint_auto_py = ament_lint_auto_py.pytest_plugin',
        ],
    },
)
