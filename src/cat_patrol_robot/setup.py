from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'cat_patrol_robot'

setup(
  name=package_name,
  version='0.0.0',
  packages=find_packages(exclude=['test']),
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='jetson',
  maintainer_email='jetson@todo.todo',
  description='Cat patrol helper nodes (mailer).',
  license='Apache-2.0',
  extras_require={'test': ['pytest']},
)
