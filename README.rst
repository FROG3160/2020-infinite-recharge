Infinite Recharge - FRC Team #3160
==================================
Repository Contents
-------------------
* /robot_src -- roboRIO source code
* /rpi_src -- Raspbery Pi coprocessor source code

Initial python setup
--------------------
Prerequisite:  Python 3.8 (64-bit)

Install the following packages with pip or pipenv:
(example:  py -3 -m pip install -U robotpy-rev or pipenv install robotpy-rev)

* pyfrc
* robotpy-ctre
* robotpy-rev

Download the base pyfrc install for the RoboRIO:

robotpy-installer download-robotpy

Download the following required packages for deployment to the RoboRIO:

robotpy-installer download-opkg <package>

* robotpy-ctre
* robotpy-rev-color

Install base package:

robotpy-installer install-robotpy

Install other required packages:
 
robotpy-installer install-opkg <package>
