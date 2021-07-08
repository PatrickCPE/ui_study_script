# UI Study Script


Test Script for the GUI vs TUI user study. Set ROS remote master for it, csv files will be stored in the execute directory. Python 2.7 interpreter, ROS Kinetic.

Make sure the button messages for the tangible interface are installed.

* git clone https://github.com/PatrickCPE/ui_study_script.git ~/catkin_ws/src/  
* roscd ui_study_script/scripts  
* mkdir study_logging  

* rosrun ui_study_script study_logger.py
* rosrun ui_study_script system_state_logger.py  

system_state_logger.py will prompt the researcher performing the user study with what object to select, and to enter 
information about the study. logs will be saved in ui_study_script/scripts/study_logging/