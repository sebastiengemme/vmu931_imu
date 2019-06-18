from setuptools import setup

# fetch values from package.xml
setup(name="vmu931_imu",
    packages=["vmu931_imu"],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'vmu931_imu_node = vmu931_imu.vmu931_imu_node:main',
            ],
    },
)
