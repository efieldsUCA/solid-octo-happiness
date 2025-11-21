from setuptools import find_packages, setup
import os
from glob import glob

package_name = "solid_octo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "configs"),
            glob(os.path.join("configs", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robopi",
    maintainer_email="robopi@todo.todo",
    description="TODO: Package description",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "diff_drive_controller = solid_octo.diff_drive_controller:main",
            "octo_pilot = solid_octo.octo_pilot:main",
        ],
    },
)
