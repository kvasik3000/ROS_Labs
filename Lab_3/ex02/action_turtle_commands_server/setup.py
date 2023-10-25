from setuptools import find_packages, setup

package_name = 'action_turtle_commands_server'

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
    maintainer='kvasik',
    maintainer_email='superadrenoline3000@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'server = action_turtle_commands_server.action_turtle_server:main',
            	'client = action_turtle_commands_server.action_turtle_client:main',
        ],
    },
)
