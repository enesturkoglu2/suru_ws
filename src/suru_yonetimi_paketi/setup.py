import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'suru_yonetimi_paketi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyalarını sisteme tanıtan satır:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enesturkoglu2',
    maintainer_email='enesturkoglu2@hotmail.com',
    description='Technofest Swarm Drone Project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Terminal komutunu kod dosyasına bağlayan satır:
            'suru_kaldir = suru_yonetimi_paketi.suru_kaldir:main',
            'suru_dans = suru_yonetimi_paketi.suru_dans:main',
          'suru_baslat = suru_yonetimi_paketi.suru_kontrol:main',
          'suru_lider_takip = suru_yonetimi_paketi.suru_lider_takip:main',
          'suru_yorunge = suru_yonetimi_paketi.suru_yorunge:main',
        ],
    },
)