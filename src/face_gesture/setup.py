from setuptools import find_packages, setup

package_name = 'face_gesture'

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
    maintainer='nrs_vision',
    maintainer_email='lexondms1@g.skku.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # [추가] ROS 2가 실행할 노드 이름과 파이썬 파일의 위치를 연결해 줍니다.
            'face_gesture_node = face_gesture.face_gesture_node:main'
        ],
    },
)
