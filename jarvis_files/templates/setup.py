from setuptools import setup, find_packages

setup(
    name="{{ component }}",
    version="0.1",
    author="MRover",
{% if src %}
    package_dir={'': 'src'},
{% endif %}
    packages=find_packages({% if src %}'src'{% endif %}),
{% if executable %}
    entry_points={
        'console_scripts': [
            '{{ component }}={{ component }}.__main__:main'  # noqa
        ]
    }
{% endif %}
)

