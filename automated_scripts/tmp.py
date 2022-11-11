import os
import re
import subprocess

REGEX_FOR_BYTE = "^b('|\")|('|\")$"
s = subprocess.check_output("/bin/bash source /opt/ros/galactic/setup.bash; ros2 node list", shell=True)
#s = subprocess.check_output("/bin/bash source /opt/ros/galactic/setup.bash")
tmp = re.sub(REGEX_FOR_BYTE, '', str(s))

p = tmp.split("\\n")
slist = list(filter(None, p))
print("output from python file\n")
print(s)
print("EOF\n")
