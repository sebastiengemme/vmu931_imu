from setuptools import setup

package_name="vmu931_imu"

# fetch values from package.xml
setup(name=package_name,
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
                     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ],
    install_requires=["setuptools"],
    zip_safe=False,
    entry_points={
        'console_scripts': [
            'vmu931_imu_node = vmu931_imu.vmu931_imu_node:main',
        ],
    },
    author='Mikael Arguedas',
    author_email='mikael@osrfoundation.org',
    maintainer='Mikael Arguedas',
    maintainer_email='mikael@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal service servers using rclpy.',
    license='Apache License, Version 2.0',
)
