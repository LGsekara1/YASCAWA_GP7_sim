import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'gp7robot_description'

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Install launch files and the self-contained Gazebo assignment worlds.
data_files.append(
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
)
data_files.append(
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf'))
)
data_files.append(
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz'))
)

# Install all files in the robot description directories
for folder in ['urdf', 'meshes', 'textures', 'config']:
    for root, dirs, files in os.walk(folder):
        if files:
            data_files.append(
                (os.path.join('share', package_name, root),
                 [os.path.join(root, f) for f in files])
            )

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=data_files,
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='GP7 robot model, kinematics, and Gazebo pick-and-place package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gp7_pick_place = gp7robot_description.pick_place:main',
            'gp7_move_joint = gp7robot_description.manual_joint:main',
        ],
    },
)
