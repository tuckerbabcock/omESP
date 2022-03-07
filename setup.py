
from setuptools import setup, find_packages

__package_name__ = "omESP"
__package_version__ = "0.1.0"


setup(
    name=__package_name__,
    version=__package_version__,
    description=("OpenMDAO interface to The Engineering Sketch Pad"
        " (https://acdl.mit.edu/esp/)"),
    author="Tucker Babcock",
    author_email="",
    zip_safe=False,
    packages = find_packages()
)