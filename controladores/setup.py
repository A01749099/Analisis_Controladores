from setuptools import find_packages, setup

package_name = 'controladores'

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
    maintainer='daniel',
    maintainer_email='daniel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turn_then_go= controladores.turn_then_go:main",
            "turn_while_go= controladores.turn_while_go:main",
            "carrot_chasing= controladores.carrot_chasing:main",
            "lyapunov= controladores.lyapunov:main",
            "turn_while_go_trajectory= controladores.turn_while_go_trajectory:main",
            "turn_then_go_trajectory= controladores.turn_then_go_trajectory:main",
            "carrot_chasing_trajectory= controladores.carrot_chasing_trajectory:main",
            "lyapunov_trajectory= controladores.lyapunov_trajectory:main",
        ],
    },
)
