from setuptools import setup

package_name = 'speaker'

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
    maintainer='Wojciech Boncela',
    maintainer_email='w.boncela@gmail.com',
    description='speaker',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speaker = speaker.speaker:main',
        ],
    },
)
