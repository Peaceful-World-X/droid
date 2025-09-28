import os

from cv2 import aruco

# Robot Params #
nuc_ip = "192.168.40.200"
robot_ip = "10.0.10.2"
laptop_ip = "192.168.40.2"
sudo_password = "Cytoderm2223"
robot_type = "fr3"  # 'panda' or 'fr3'
robot_serial_number = "290102-40602"

# Camera ID's #
hand_camera_id = ""
varied_camera_1_id = ""
varied_camera_2_id = ""

# Charuco Board Params #
CHARUCOBOARD_ROWCOUNT = 9
CHARUCOBOARD_COLCOUNT = 14
CHARUCOBOARD_CHECKER_SIZE = 0.020
CHARUCOBOARD_MARKER_SIZE = 0.016
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_100)

# Ubuntu Pro Token (RT PATCH) #
ubuntu_pro_token = "C12WmUrvpKfaxvyGy1PBKVF7EYgem8"

# Code Version [DONT CHANGE] #
droid_version = "1.3"