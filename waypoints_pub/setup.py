from setuptools import find_packages, setup

package_name = 'waypoints_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/my_path.xml', 'resource/my_path2.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alvaro',
    maintainer_email='alvaro.herraez.duque@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "waypoints_publisher = waypoints_pub.send_waypoints_xml:main"
        ],
    },
)
