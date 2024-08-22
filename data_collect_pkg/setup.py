from setuptools import find_packages, setup

package_name = 'data_collect_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arata22',
    maintainer_email='ky.37f.9850@s.thers.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_aggregator_node = data_collect_pkg.data_collect_node:main'
        ],
    },
)
