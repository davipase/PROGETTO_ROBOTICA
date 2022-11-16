#################################h##########################################################
#                                                                                         #
# Set up paths for the Object Detection Metrics                                           #
#                                                                                         #
# Developed by: Rafael Padilla (rafael.padilla@smt.ufrj.br)                               #
#        SMT - Signal Multimedia and Telecommunications Lab                               #
#        COPPE - Universidade Federal do Rio de Janeiro                                   #
#        Last modification: May 24th 2018                                                 #
###########################################################################################

import sys
import os


def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)


currentPath = os.path.dirname(os.path.realpath(__file__))

# Add lib to PYTHONPATH
cvPath = os.path.join(currentPath, 'ur_grippertest_dev/src/cv_ros/src/cv_lib/src')
#modelPath = os.path.join(currentPath, 'ur_grippertest_dev/src/cv_ros/src/cv_lib/models')

add_path(currentPath)
add_path(cvPath)
