from setuptools import find_packages
from setuptools import setup

package_name = 'ament_clang_tidy'

setup(
    name=package_name,
    version='0.20.6',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['pyyaml'],
    package_data={'': [
        'configuration/.clang-tidy',
        'py.typed'
    ]},
    zip_safe=False,
    author='John Shepherd',
    author_email='john@openrobotics.org',
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
    description='Check C++ code style using clang-tidy.',
    long_description="""\
The ability to check code against style conventions using clang-tidy
and generate xUnit test result files.""",
    license='Apache License, Version 2.0, BSD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ament_clang_tidy = ament_clang_tidy.main:main',
        ],
    },
)
