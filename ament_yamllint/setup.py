from setuptools import find_packages
from setuptools import setup


package_name = 'ament_yamllint'

setup(
    name=package_name,
    version='0.20.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['PyYAML', 'setuptools'],
    package_data={'': [
        'configuration/yamllint.yaml',
    ]},
    zip_safe=False,
    author='Woojin Jung',
    author_email='frozenreboot@gmail.com',
    maintainer='Scott K Logan, Alejandro Hernández Cordero',
    maintainer_email='logans@cottsay.net, ahcorde@gmail.com',
    url='https://github.com/ament/ament_lint',
    download_url='https://github.com/ament/ament_lint/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Check YAML style using YAMLlint.',
    long_description="""\
The ability to check YAML against style conventions using YAMLlint
and generate xUnit test result files.""",
    license='Apache License, Version 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ament_yamllint = ament_yamllint.main:main',
        ],
        'pytest11': [
            'ament_yamllint = ament_yamllint.pytest_marker',
        ],
    },
)
