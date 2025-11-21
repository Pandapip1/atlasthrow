from setuptools import find_packages, setup
from glob import glob

package_name = 'demos'

# Create a mapping of other files to be copied in the src -> install
# build.  This is a list of tuples.  The first entry in the tuple is
# the install folder into which to place things.  The second entry is
# a list of files to place into that folder.
otherfiles = [
    ('share/' + package_name + '/launch', glob('launch/*')),
    ('share/' + package_name + '/rviz',   glob('rviz/*')),
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
    description='The 133a Project Code Demos',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'boolean_publisher  = demos.boolean_publisher:main',
            'float_publisher    = demos.float_publisher:main',
            'point_publisher    = demos.point_publisher:main',
            'pose_publisher     = demos.pose_publisher:main',
            'subscriber         = demos.subscriber:main',
            'balldemo           = demos.balldemo:main',
            'fencedemo          = demos.fencedemo:main',
            'interactivedemo    = demos.interactivedemo:main',
            'pirouette          = demos.pirouette:main',
            'pirouetteandwave   = demos.pirouetteandwave:main',
        ],
    },
)
