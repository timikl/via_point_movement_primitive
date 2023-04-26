from distutils.core import setup
import setuptools

setup(
    name='robotblockset_python',
    version='1.0',
    url='https://repo.ijs.si/leon/robotblockset_python',
    author='Leon Zlajpah',
    author_email='leon.zlajpah@ijs.si',
    description=('Robotblockset_python allows control of various different robots and provides common functions such as transformations.'),
    license='BSD',
    packages=['robotblockset_python'],
    test_suite='tests',
    install_requires=['numpy', 'numpy-quaternion'],
    keywords="robot blockset python robots control transformations trajectory",
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.1',
        'Programming Language :: Python :: 3.2',
        'Programming Language :: Python :: 3.3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
    ]
)
