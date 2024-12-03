from setuptools import setup

package_name = 'sheep_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launch.py']),
        ('share/' + package_name, ['launch/launch_rviz.py']),
        ('share/' + package_name, ['launch/launch_sim.py']),
        ('share/' + package_name, ['config/sheep_simulation_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='Sheep and wolf simulation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sheep_node = sheep_simulation.sheep_node:main',
            'wolf_node = sheep_simulation.wolf_node:main',
            'master_node = sheep_simulation.master_node:main'
        ],
    },
)
