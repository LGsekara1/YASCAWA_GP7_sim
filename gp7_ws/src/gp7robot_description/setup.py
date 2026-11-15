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

# Install launch files
data_files.append(
    (os.path.join('share', package_name, 'launch'),
     glob('launch/*.py'))
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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='GP7 Robot description package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
