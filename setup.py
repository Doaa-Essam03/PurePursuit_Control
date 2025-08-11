from setuptools import find_packages, setup

package_name = 'pure_pursuit_pkg'

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
    maintainer='doaa',
    maintainer_email='doaaessa559@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker_2 = pure_pursuit_pkg.talker:my_func ' ,
            'subscriber_2 = pure_pursuit_pkg.listener_node:my_func ' ,
            'pps_01 = pure_pursuit_pkg.pps_01:main ',
            'pps_path = pure_pursuit_pkg.pps_path:main ' , 
            'pps_compete = pure_pursuit_pkg.pps_cdc_practice:main ' 
        ],
    },
)
