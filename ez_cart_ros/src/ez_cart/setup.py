from setuptools import setup

package_name = 'ez_cart'

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
    maintainer='rileytuttle',
    maintainer_email='rileytuttle@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0 (maybe)',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ez_cart.intent_publisher:main',
            'listener = ez_cart.intent_listener:main',
        ],
    },
)
