from setuptools import find_packages
from setuptools import setup

package_name = 'ament_bandit'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Florencia Cabral',
    author_email='florencia.cabralberenfus@canonical.com',
    maintainer='Florencia Cabral',
    maintainer_email='florencia.cabralberenfus@canonical.com',
    url='https://github.com/ament/ament_lint',
    download_url='https://github.com/ament/ament_lint/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Static code analysis on Python code using Bandit.',
    long_description="""\
The ability to perform static code analysis on Python code using Bandit
and generate xUnit test result files.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ament_bandit = ament_bandit.main:main',
        ],
    },
)

