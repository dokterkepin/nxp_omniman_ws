from glob import glob

from setuptools import setup

package_name = 'omniman_cyclo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/native', glob('native/*.py') + glob('native/*.sh')),
        (f'share/{package_name}/bt', glob('bt/*.py')),
        (f'share/{package_name}/robot_configs', glob('robot_configs/*.yaml')),
        (f'share/{package_name}/trees', glob('trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dokterkepin',
    maintainer_email='btw.sneakythief33@gmail.com',
    description='Cyclo Intelligence for omniman',
    license='Apache-2.0',
)
