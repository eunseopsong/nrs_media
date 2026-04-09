from setuptools import setup

package_name = 'nrs_hand_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eunseop',
    maintainer_email='eunseop@example.com',
    description='MediaPipe hand landmarks to dual-arm hand teleoperation targets',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'hand_teleop_node = nrs_hand_teleop.hand_teleop_node:main',
        ],
    },
)