#!/usr/bin/python

from setuptools import setup, find_packages

setup(
        name = "labjacku12",
        version = "0.4.1",
        author = "Christopher Kolbe",
        author_email = "kontakt@christopherkolbe.de",
        description = "LabJack U12 driver with Python 3 and libusb-1.0 Support.",
        license = "BSD",
        install_requires = [
            'libusb1>=1.8'
        ],
        dependency_links = [
            "http://code.enthought.com/enstaller/eggs/source",
            ],
        url = "https://github.com/chkolbe/labjacku12",
        packages = find_packages(),
        test_suite = "tests", #.labjacku12_tests.LabjackU12Tests",
        #scripts = ['scripts/u12_scope.py', 'scripts/u12_rrd.py'],
        include_package_data = True,
        zip_safe = True,
        )
