from setuptools import find_packages, setup

package_name = "py_example"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arthemis",
    maintainer_email="alejandroamar66@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"simple_parameter={package_name}.single_parameter:main",
            f"simple_services_server={package_name}.simple_services_server:main",
            f"simple_services_client={package_name}.simple_services_client:main",
            f"simple_action_server={package_name}.simple_action_server:main"

        ],
    },
)
