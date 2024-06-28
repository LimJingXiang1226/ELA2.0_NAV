from setuptools import find_packages, setup

package_name = 'ella2_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'tf_transformations',
    ],
    zip_safe=True,
    maintainer='ljx',
    maintainer_email='ljx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	"pose_fusion_node = ella2_pose.pose_fusion_node:main"
        ],
    },
)
