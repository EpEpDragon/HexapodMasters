from setuptools import setup

package_name = 'pub_key_lr'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='epep',
    maintainer_email='epep@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'key_publish = pub_key_lr.publisher:main',
        'angle_listener = pub_key_lr.subscriber:main'
        ],
    },
)
