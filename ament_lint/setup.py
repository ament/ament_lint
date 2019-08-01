from setuptools import find_packages
from setuptools import setup

setup(
    name='ament_lint',
    version='0.7.6',
    packages=find_packages(exclude=['test']),
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
    description='Providing common API for ament linter packages.',
    long_description="""\
Providing common API for ament linter packages, e.g. the `linter` marker for
pytest.""",
    license='Apache License, Version 2.0',
    entry_points={
        'pytest11': [
            'ament_lint = ament_lint.pytest_marker',
        ],
    },
)
