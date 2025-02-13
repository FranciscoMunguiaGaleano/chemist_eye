#!/usr/bin/env python
import os
import time
import subprocess


# Absolute path to the script
bash_script = '/home/francisco/catkin_ws/src/chemist_eye/scripts/chemisteye_one_warning.bash'

# Ensure the script is executable
os.chmod(bash_script, 0o755)

# Run the bash script
try:
    subprocess.check_call([bash_script])
except subprocess.CalledProcessError as e:
    print(f"Error executing the bash script: {e}")

