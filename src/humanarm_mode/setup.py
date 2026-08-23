from setuptools import setup
from glob import glob

package_name = "humanarm_mode"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jhj0129",
    maintainer_email="noreply@example.com",
    description="DROK ARM HumanArm mode: power-on session calibration, smooth 1 kHz control, and IK pick demo.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "humanarm_motor_monitor = humanarm_mode.raw_monitor:main",
            "humanarm_capture_boot_reference = humanarm_mode.boot_reference:main",
            "humanarm_session_control = humanarm_mode.session_control:main",
            "humanarm_legacy_home_once = humanarm_mode.legacy_home_once:main",
            "humanarm_pick_60cm = humanarm_mode.pick_60cm:main",
        ],
    },
)
