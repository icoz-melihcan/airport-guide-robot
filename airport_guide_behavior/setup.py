import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'airport_guide_behavior'
setup(
name=package_name,
version='0.1.0',
packages=find_packages(exclude=['test']),
data_files=[
('share/ament_index/resource_index/packages', ['resource/' + package_name]),
('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
],
install_requires=['setuptools'],
zip_safe=True,
maintainer='Melih Can İçöz',
maintainer_email='icozmelihcan@gmail.com',
description='Havalimani rehber robotu: algilama ve davranis dugumleri',
license='Apache-2.0',
entry_points={
'console_scripts': [
'guide_node = airport_guide_behavior.guide_node:main',
'algilama_node = airport_guide_behavior.algilama_node:main',
],
},
)
