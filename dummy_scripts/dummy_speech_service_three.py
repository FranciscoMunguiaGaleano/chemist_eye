#!/usr/bin/env python

import os
import subprocess
import rospy
from chemist_eye.srv import RunBashScript, RunBashScriptResponse  

# Path to the bash script
BASH_SCRIPT_PATH = '/home/francisco/catkin_ws/src/chemist_eye/scripts/chemisteye_three_warning.bash'

def run_bash_script(req):
    """
    Service callback to execute the bash script.
    """
    rospy.logwarn("Remember to wear PPE.")
    return RunBashScriptResponse(success=True, message="Speech service three executed successfully.")

def bash_script_service():
    """
    Initializes the bash script service node.
    """
    rospy.init_node('warning_speech_service_three')
    service = rospy.Service('run_speech_service_three', RunBashScript, run_bash_script)
    rospy.loginfo("Speech service three is ready.")
    rospy.spin()

if __name__ == "__main__":
    bash_script_service()
