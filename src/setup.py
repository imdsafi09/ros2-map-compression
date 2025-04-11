from setuptools import setup

package_name = 'map_compression'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Compress and decompress OccupancyGrid maps in real time for SLAM Toolbox.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compressed_map_publisher = map_compression.compressed_map_publisher:main',
            'decompressed_map_subscriber = map_compression.decompressed_map_subscriber:main',
            'map_comparator = map_compression.map_comparator:main',
        ],
    },
)

