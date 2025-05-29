from glob import glob
from setuptools import find_packages, setup
package_name = "task_manager"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        # ament index
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # package manifest
        (f"share/{package_name}", ["package.xml"]),
        # launch files
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        # configuration and controllers
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    maintainer="wahid",
    maintainer_email="wahidas@stud.ntnu.no",
    description="Task manager for UR3 cube detection and pointing",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_manager_node = task_manager.task_manager_node:main",
            "integrated_test = test.integrated_test:main",  #
       ],
    },
)