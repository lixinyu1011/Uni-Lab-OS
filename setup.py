from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'unilabos'

setup(
    name=package_name,
    version='0.8.0',
    packages=find_packages(),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    #     # (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    #     # (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    #     # (os.path.join('share', package_name, 'config'), glob('config/*'))
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junhan Chang',
    maintainer_email='changjh@pku.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "unilab = unilabos.app.main:main",
        ],
    },
)
