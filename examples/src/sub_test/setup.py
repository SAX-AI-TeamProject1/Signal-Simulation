from setuptools import find_packages, setup

package_name = 'sub_test'

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
    maintainer='sho',
    maintainer_email='shp9826@naver.com',
    description='Subscribe Topic Test',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sub_node=sub_test.topic_sub_base:main'
        ],
    },
)
