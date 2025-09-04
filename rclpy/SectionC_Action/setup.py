from setuptools import find_packages, setup

package_name = 'SectionC_Action'

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
            'circle_action_server = SectionC_Action.circle_action_server:main',
            'circle_action_client = SectionC_Action.circle_action_client:main',
        ],
    },
)
