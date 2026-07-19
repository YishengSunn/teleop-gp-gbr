from setuptools import find_packages, setup

package_name = 'geo_gp_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    extras_require={'mujoco': ['mujoco>=3.2']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='sunyisheng48@gmail.com',
    description='Offline tests for Geo-GP fusion behavior.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offline_fusion_test = geo_gp_test.offline_fusion_test:main',
        ],
    },
)
