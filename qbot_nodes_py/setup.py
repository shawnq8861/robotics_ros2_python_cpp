from setuptools import find_packages
from setuptools import setup

package_name = 'qbot_nodes_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Esteve Fernandez',
    author_email='shawnquinn861@gmail.com',
    maintainer='Shawn Quinn',
    maintainer_email='shawnquinn861@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Python nodes for QBot robot control, and for building other Python nodes.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = qbot_nodes_scripts_py.listener:main',
            'talker = qbot_nodes_scripts_py.talker:main'
        ],
    },
)
