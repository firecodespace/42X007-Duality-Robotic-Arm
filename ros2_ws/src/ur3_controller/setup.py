from setuptools import find_packages, setup

package_name = 'ur3_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # INSTALL CONFIG FOLDER
        ('share/' + package_name + '/config', [
            'ur3_controller/config/tools.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evan',
    maintainer_email='evan.goldman@duality.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ur3_controller_node = ur3_controller.ur3_controller_node:main',
        ],
    },
)
