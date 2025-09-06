from setuptools import find_packages, setup

package_name = 'section_a_pub_sub'

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
    maintainer='porsche',
    maintainer_email='porsohani@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_logger = section_a_pub_sub.odom_logger:main',  # Add this line
            'circle_publisher = section_a_pub_sub.circle_publisher:main'
        ],
    },
)
