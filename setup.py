#!/usr/bin/python

from setuptools import setup, find_packages

setup(
        name = "labjacku12",
        version = "0.1",
        author = "Robert Jordens",
        author_email = "jordens@phys.ethz.ch",
        description = "LabJack U12 driver",
        license = "BSD",
        #install_requires = [],
        #extras_require = {
        #    "gadgets": ["numpy", "Chaco2",],
        #    },
        dependency_links = [
            "http://code.enthought.com/enstaller/eggs/source",
            ],
        url = "http://www.phys.ethz.ch/~robertjo/labjacku12",
        packages = find_packages(),
        # test_suite = "tests", #.labjacku12_tests.LabjackU12Tests",
        scripts = ['scripts/*.py'],
        include_package_data = True,
        zip_safe = True,
        )
