from setuptools import find_packages, setup

package_name = 'inverse_differential_kinematics'

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
    maintainer='martijn',
    maintainer_email='mbrummelhuis@gmail.com',
    description='Package containing inverse differential kinematics algorithms for ATS project',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inverse_differential_kinematics = inverse_differential_kinematics.inverse_differential_kinematics:main'
        ],
    },
)
