from setuptools import setup
package_name='asv_operator_ui'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['flask', 'setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Flask operator UI for ASV demo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'operator_ui = asv_operator_ui.app:main',
        ],
    },
)
