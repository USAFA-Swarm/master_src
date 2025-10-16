from setuptools import find_packages, setup

package_name = 'move2hover'

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
    maintainer='m3',
    maintainer_email='ty13hubert@gmail.com',
    description='Move to Hover Position',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move2hover = move2hover.move2hover:main',
        ],
    },
)