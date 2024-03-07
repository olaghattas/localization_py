from setuptools import setup
# import os
# from glob import glob

package_name = 'localization_aptags'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # # ('share/' + package_name, ['config/transforms.json']),
        # ('share/' + package_name, ['config/transform_matrices_dictionary.yaml']),
        #(os.path.join('share', package_name, 'yolodata'), glob('yolodata/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ola',
    maintainer_email='olaghattas@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_pose = localization_aptags.get_pose:main',
            'pid_aptags = localization_aptags.pid_aptags:main',
            'localize = localization_aptags.localize:main',
        ],
    },
)
