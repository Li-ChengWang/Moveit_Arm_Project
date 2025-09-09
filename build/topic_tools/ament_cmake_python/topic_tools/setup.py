from setuptools import find_packages
from setuptools import setup

setup(
    name='topic_tools',
    version='1.1.1',
    packages=find_packages(
        include=('topic_tools', 'topic_tools.*')),
)
