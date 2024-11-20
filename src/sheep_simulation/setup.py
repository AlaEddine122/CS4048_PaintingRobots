from setuptools import setup

package_name = 'sheep_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
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
        ],
    },
)
