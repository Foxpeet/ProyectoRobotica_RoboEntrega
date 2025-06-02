from setuptools import setup

package_name = 'robo_entrega_qr_scanner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'pyzbar'],
    zip_safe=True,
    maintainer='Jordan D. Phillips Fonta',
    maintainer_email='your_email@example.com',
    description='QR Code scanner ROS2 node for RoboEntrega project.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_code_scanner = robo_entrega_qr_scanner.qr_code_scanner:main',
        ],
    },
)
