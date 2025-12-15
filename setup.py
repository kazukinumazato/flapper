from setuptools import setup, find_packages

setup(
    name="flapper",
    version="0.0.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
)