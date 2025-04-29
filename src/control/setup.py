from setuptools import setup
package_name = 'asv_control'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Shared control nodes for ASV demo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ai_planner = asv_control.ai_planner:main',
            'switcher_node = asv_control.switcher_node:main',
        ],
    },
)
