import setuptools

NAME_PACKAGE = "nimbro-ids-ros2"

setuptools.setup(
    name=NAME_PACKAGE,
    version="0.0.1",
    packages=setuptools.find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{NAME_PACKAGE}"]),
        (f"share/{NAME_PACKAGE}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Bertan Karacora",
    maintainer_email="bertan.karacora@gmail.com",
    description="ROS2 package for IDS camera",
    license="MIT",
    entry_points={
        "console_scripts": [
            "publish = nimbro-ids-ros2.publish_camera_ids:main",
        ],
    },
)
