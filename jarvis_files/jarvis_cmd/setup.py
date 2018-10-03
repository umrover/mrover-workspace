from setuptools import setup, find_packages

setup(
    name="jarvis",
    version="2.0",
    author="MRover",
    package_dir={'': 'src'},
    packages=find_packages('src'),
    entry_points={
        'console_scripts': [
            'jarvis=jarvis.__main__:main'
        ]
    }
)
