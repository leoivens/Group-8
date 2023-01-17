from setuptools import setup

package_name = 'tuto_move'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bot',
    maintainer_email='bot@mb6.imt-nord-europe.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'reactive_move = tuto_move.reactive_move:move',
        'scan_echo = tuto_move.scan_echo:main',
        'move_1m = tuto_move.move_1m:main',
        'camera = tuto_move.camera:main',
        'test = tbot_pytools.test:move'
        ],
    },
)
