#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=[
    "cage_site_db", "cage_site_db.importers", "cage_site_db.processing"
], package_dir={"": "."}))
