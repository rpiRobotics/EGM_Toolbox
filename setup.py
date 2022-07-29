from setuptools import setup

setup(
    name='egm_toolbox',
    version='0.0.1',
    description='EGM Tools',
    py_modules=['rpi_abb_irc5','EGM_toolbox','egm_pb2'],
    install_requires=[
        'bs4',
        'numpy'
    ]
)