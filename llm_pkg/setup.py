from setuptools import find_packages, setup
from glob import glob

import os

package_name = 'llm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),  # SRV 파일 추가
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # launch 폴더의 파일들 추가
        (os.path.join('share', package_name, 'jsons'), glob('llm_pkg/jsons/*')) # jsons 파일 추가
    ],
    install_requires=[
        'setuptools', 
],
    zip_safe=True,
    maintainer='god',
    maintainer_email='ehd2610@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wakeup_node = llm_pkg.wakeup_node:main',
            'stream_stt_node = llm_pkg.stream_stt_node:main',
            'daya_subscription_node = llm_pkg.daya_subscription_node:main',
            'propose_node = llm_pkg.propose_node:main',
        ],
    },
)
