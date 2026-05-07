from glob import glob
import os
from setuptools import setup

package_name = "pfe"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pfe",
    maintainer_email="pfe@todo.todo",
    description="Compatibility launch package for the renamed match package",
    license="TODO: License declaration",
)
