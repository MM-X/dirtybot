from setuptools import setup

package_name = 'originbot_demo'

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
    maintainer='GuYueHome',
    maintainer_email='support@ps-micro.com',
    description='OriginBot Demo',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_circle    = originbot_demo.draw_circle:main',
            'echo_odom      = originbot_demo.echo_odom:main',
            'echo_status    = originbot_demo.echo_status:main',
            'control_buzzer = originbot_demo.control_buzzer:main',
            'control_led    = originbot_demo.control_led:main',
        ],
    },
)
