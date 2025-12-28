from setuptools import setup

package_name = 'stereo_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=['stereo_perception'],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'rectification_node = stereo_perception.rectification_node:main',
            'disparity_node     = stereo_perception.disparity_node:main',
            'depth_node         = stereo_perception.depth_node:main',
        ],
    },
)
