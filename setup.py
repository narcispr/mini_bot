from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mini_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools', 'numpy', 'std_srvs'],
    zip_safe=True,
    maintainer='narcis',
    maintainer_email='narcis@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mini_bot_node = mini_bot.mini_bot_node:main',
            'navigation_node = mini_bot.navigation_node:main',
            'controller_node = mini_bot.controller_node:main'
        ],
    },
)
