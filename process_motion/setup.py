from glob import glob
import os

from setuptools import setup


package_name = 'process_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=['process_motion'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join(package_name, package_name)]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name, 'config'), glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aron Svastits',
    maintainer_email='svastits1@gmail.com',
    description='Postprocessing of recorded motions',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'script_parser = process_motion.script_parser:main'
        ],
    },
)
