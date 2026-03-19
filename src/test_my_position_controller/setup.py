from setuptools import find_packages, setup

package_name = 'test_my_position_controller'

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
    maintainer='gaosn',
    maintainer_email='17864235234@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_command = test_my_position_controller.publish_command:main',
            'rec_state = test_my_position_controller.rec_state:main',
        ],
    },
)
