from setuptools import setup
import os
from glob import glob

package_name = 'robo_entrega_nav_to_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='equipo1',
    maintainer_email='equipo1@gmail.com',
    description='Suscriptor a /navigate_goal que controla la navegación del robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_to_pose_subscriber = robo_entrega_nav_to_pose.nav_to_pose_subscriber:main'
        ],
    },
)
