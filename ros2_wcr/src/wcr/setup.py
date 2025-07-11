from setuptools import find_packages, setup

package_name = 'wcr'

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
    maintainer='marija',
    maintainer_email='marija@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'linear_trajectory_node = wcr.linear_trajectory_node:main',
            	'linear_tr_node = wcr.linear_tr_node:main',
        	'circular_trajectory_node = wcr.circular_trajectory_node:main',
        	'square_trajectory_node = wcr.square_trajectory_node:main',
        	'plot_linear = plot.plot_linear:main',
            	'desired_linear_trajectory = wcr.desired_linear_trajectory:main',
            	'desired_circular_trajectory = wcr.desired_circular_trajectory:main',
            	'eight_trajectory_node = wcr.eight_trajectory_node:main',
            	'desired_eight_trajectory_node = wcr.desired_eight_trajectory_node:main',
            	'policy_node = wcr.policy_node:main',
            	'ppo_agent = wcr.ppo_agent:main',
            	'desired_trained_eight = wcr.desired_trained_eight:main',
        ],
    },
)
