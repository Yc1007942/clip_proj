from glob import glob
from setuptools import setup, find_packages

package_name = 'system1'

setup(
    name=package_name,
    version='0.0.0',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'numpy<2',
        'scipy>=1.15.2',
        'pyads>=3.5.0',
        'ur_rtde>=1.6.1',
    ],
    entry_points={
        'console_scripts': [
            'run_system1 = system1.__main__:main',
            'wacoh_force_sensor_node = system1.sensor_nodes.wacoh_force_sensor_node:main',
        ],
    },
    zip_safe=True,
    maintainer='Yusei Suzuki',
    maintainer_email='suzuki.yusei001@jp.panasonic.com',
    description='System1',
    license='License',
    tests_require=['pytest'],
)
