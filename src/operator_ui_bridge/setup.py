from setuptools import setup
package_name='operator_ui'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['flask', 'setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='fruktin03@gmail.com',
    description='Flask operator UI for ASV demo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'operator_ui = operator_ui.app:main',
        ],
    },
)
