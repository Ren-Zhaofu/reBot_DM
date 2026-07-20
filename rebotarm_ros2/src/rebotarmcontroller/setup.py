from setuptools import find_packages, setup


package_name = "rebotarmcontroller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Portable core utilities and controller nodes for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "check_hardware_config = rebotarmcontroller.hardware_config:main",
            "cartesian_bridge = rebotarmcontroller.cartesian_bridge:main",
            "inspect_sdk = rebotarmcontroller.sdk_loader:main",
            "inspect_hardware = rebotarmcontroller.hardware_manager:main",
            "read_only_check = rebotarmcontroller.read_only_check:main",
            "joint_state_node = rebotarmcontroller.joint_state_node:main",
            "moveit_real_smoke = rebotarmcontroller.moveit_real_smoke:main",
        ],
    },
)
