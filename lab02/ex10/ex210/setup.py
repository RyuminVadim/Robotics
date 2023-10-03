from setuptools import find_packages, setup

package_name = 'ex210'

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
    maintainer='vadim',
    maintainer_email='dflbr201@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
	'console_scripts': [
	'text_to_cmd_vel = ex210.text_to_cmd_vel:main',],
    },
)
