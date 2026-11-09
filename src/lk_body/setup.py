from setuptools import find_packages, setup

package_name = "lk_body"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/mimics.json", "config/hardware.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jetson",
    maintainer_email="jetson@savelij.local",
    description="Savelij plays LogosKittens teacher audio on his speaker. He does not record.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lk_body_node = lk_body.node:main",
            "lk_bot_mode = lk_body.mode:main",
        ],
    },
)
