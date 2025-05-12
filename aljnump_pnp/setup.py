from glob import glob
from setuptools import setup

package_name = "aljnump_pnp"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yuth",
    maintainer_email="yuth@todo.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            f"pnp_perception = {package_name}.pnp_perception:main",
            f"pnp_real_run = {package_name}.pnp_real_run:main",
        ],
    },
)
