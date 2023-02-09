from setuptools import setup

package_name = 'qr_verify'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Josué Fleitas',
    maintainer_email='josue.fle.sanc@gmail.com',
    description='A simple verifier library to authenticate a delivery',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_verify = qr_verify.qr_verify:main'
        ],
    },
)
