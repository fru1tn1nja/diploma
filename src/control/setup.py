from setuptools import setup
package_name = 'control'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='fruktin03@gmail.com',
    description='Shared control nodes for ASV demo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ai_planner = control.ai_planner:main',
            'switcher_node = control.switcher_node:main',
        ],
    },
)
