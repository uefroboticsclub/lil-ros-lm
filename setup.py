from setuptools import setup
from glob import glob
import os

package_name = 'rosgpt'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'webapp'), glob('webapp/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Glory',
    author_email='olusolagloryolamide@gmail.com',
    maintainer='UEF Robotics',
    maintainer_email='olusolagloryolamide@gmail.com',
    keywords=['ROS', 'LLM'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Creative Commons Attribution-NonCommercial License (CC BY-NC)',
        'Programming Language :: Python :: 3.10',
    ],
    description='A ROS2 package for natural language processing and semantic interpretation of unstructured textual commands in human-robot interaction contexts.',
    license='Creative Commons Attribution-NonCommercial (CC BY-NC)',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosgpt = rosgpt.rosgpt:main',
            'rosgptparser_turtlesim = rosgpt.rosgptparser_turtlesim:main',
            'rosgpt_client_node = rosgpt.rosgpt_client_node:main',
        ],
    },
)
