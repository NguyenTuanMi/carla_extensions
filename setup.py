import os

from setuptools import setup


package_name = 'carla_extensions'

setup(
	name=package_name,
	version='0.1.0',
	packages=[package_name],
	package_dir={'': 'src'},
	data_files=[
		(
			'share/ament_index/resource_index/packages',
			['resource/' + package_name],
		),
		('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	entry_points={
		'console_scripts': [
			'map_pose_publisher = carla_extensions.carla_odom_node:main',
		],
	},
)
