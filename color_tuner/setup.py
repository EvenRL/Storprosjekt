from setuptools import find_packages, setup

package_name = 'color_tuner'

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
    maintainer='even',
    maintainer_email='52103879+EvenRL@users.noreply.github.com',
    description='GUI for easy tuning of hsv color ranges',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_tuner = color_tuner.color_tuner:main',
        ],
    },
)
