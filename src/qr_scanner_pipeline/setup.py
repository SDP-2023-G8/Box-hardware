from setuptools import setup
import os
from glob import glob

package_name = 'qr_scanner_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wojciech Boncela',
    maintainer_email='w.boncela@gmail.com',
    description='Contains a node that decodes and published messages from QR codes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_scanner = qr_scanner_pipeline.qr_scanner:main'
        ],
    },
)
