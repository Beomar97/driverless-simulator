from setuptools import setup

package_name = 'dataplot_pkg'

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
    maintainer='user',
    maintainer_email='vicengui@students.zhaw.ch',
    description='Package for plotting data via ROS2',
    license='GPL-2.0 License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dataplot = dataplot_pkg.dataplot:main'
        ],
    },
)
