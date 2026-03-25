from setuptools import find_packages, setup

package_name = 'geo_gp_prediction'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yisheng Sun',
    maintainer_email='sunyisheng48@gmail.com',
    description='GP-based skill prediction for robot trajectories.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prediction_node = geo_gp_prediction.prediction_node:main',
        ],
    },
)
