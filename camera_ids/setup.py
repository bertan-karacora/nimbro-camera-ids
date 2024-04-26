import setuptools

NAME_PACKAGE = "camera_ids"

setuptools.setup(
    name=NAME_PACKAGE,
    version="0.0.1",
    packages=setuptools.find_namespace_packages(),
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
            "publish = camera_ids.publish_camera_ids:main",
        ],
    },
)
