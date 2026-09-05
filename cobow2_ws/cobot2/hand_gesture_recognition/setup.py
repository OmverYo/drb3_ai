from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'hand_gesture_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #ljs
        (os.path.join('share', package_name, 'resource'), glob('resource/*.*'))
        ##
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='jungsub27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            #ljs
            'get_command = hand_gesture_recognition.get_command:main',
            ##
        ],
    },
)
