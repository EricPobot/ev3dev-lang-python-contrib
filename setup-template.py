from setuptools import setup

setup(
    name='python-ev3dev-contrib',
    namespace_packages=['ev3dev'],
    version='%(version)s',
    description='Python ev3dev contribution packages',
    author='Eric Pascual',
    author_email='eric@pobot.org',
    license='MIT',
    include_package_data=True,
    packages=['ev3dev', 'ev3dev.contrib'],
    install_requires=['python-ev3dev-ep'],
    package_dir={'': 'src'},
    )

