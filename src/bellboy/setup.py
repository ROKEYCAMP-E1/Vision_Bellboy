# setup.py
from setuptools import find_packages, setup

package_name = 'bellboy'

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
    maintainer='rokey',
    maintainer_email='kim3he@gamil.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topview_node = bellboy.topview_camera.topview_cam_node:main', 
            'amr_node = bellboy.amr.amr_node:main',
            'robot_interface_node = bellboy.UI.robot_interface:main',
            'interface_node = bellboy.flask.interface_node:main',
        ],
    },
)
