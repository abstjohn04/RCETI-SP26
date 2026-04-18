import glob
import os

from setuptools import find_packages, setup

package_name = 'rceti_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob.glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='Computer vision and visual servoing bridge for the RCETI continuum robot.',
    license='Apache-2.0',
    
    entry_points={
        'console_scripts': [
            'vision_bridge = rceti_vision.vision_bridge:main',
            'mock_camera = rceti_vision.mock_camera:main'
        ],
    },
)
