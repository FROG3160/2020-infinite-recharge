Infinite Recharge - FRC Team #3160
==================================
Raspberry Pi Source Code
------------------------

**File Locations**

- /home/pi/claunch.sh
This bash script is launched by /etc/rc.local at the end of the boot sequence.  It launches the python file for camera target processing.

- /home/pi/pivi
This directory holds all of the python files used to process vision targets and put them on NetworkTables for communication with the RoboRio.