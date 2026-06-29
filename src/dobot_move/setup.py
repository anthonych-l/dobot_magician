from setuptools import find_packages, setup

package_name = 'dobot_move'

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
    maintainer='ubuntu',
    maintainer_email='anthony.chamba.05@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_place = dobot_move.pick_place:main',
            'laser = dobot_move.laser_square_test:main',
            'laser_engraver = dobot_move.laser_engraver:main',
        ],
    },
)
