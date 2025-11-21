from setuptools import find_packages, setup
from glob import glob

package_name = 'hw3code'

# Create a mapping of other files to be copied in the src -> install
# build.  This is a list of tuples.  The first entry in the tuple is
# the install folder into which to place things.  The second entry is
# a list of files to place into that folder.
otherfiles = [
    ('share/' + package_name + '/urdf', glob('urdf/*')),
]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]+otherfiles,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133a HW3 Code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw3p4              = hw3code.hw3p4:main',
            'hw3p4alternate     = hw3code.hw3p4alternate:main',
            'hw3p5              = hw3code.hw3p5:main',
            'hw3p6              = hw3code.hw3p6:main',
        ],
    },
)
