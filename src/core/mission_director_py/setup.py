from setuptools import find_packages, setup

package_name = 'mission_director_py'

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
    maintainer='orangepi',
    maintainer_email='mbrummelhuis@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'esa_demo_md = mission_director_py.mission_director_py:main',
            'dry_ats_md = mission_director_py.dry_ats_mission_director_py:main',
            'sim_mission_director = mission_director_py.sim_mission_director_py:main',
            'flight_mission_director_takeoff_land = mission_director_py.flight_mission_director_takeoff_land:main',
        ],
    },
)
