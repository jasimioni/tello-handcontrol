from setuptools import find_packages, setup

package_name = 'tello_mediapipe'

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
    maintainer='Joao Andre Simioni',
    maintainer_email='jasimioni@gmail.com',
    description='Controll Tello using hands',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gestures = tello_mediapipe.read_gestures:main',
        ],
    },
)
