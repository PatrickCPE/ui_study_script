#!/usr/bin/env python

# coding=utf-8
# Test Script for the GUI vs TUI study

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt64
from ur_scooter_msgs.msg import BtnState
import datetime
import os


class SuccessLogger:
    """
    Script to manage both logging of the Scooter's TUI and GUI system state. Logs time stamp as well as button presses
    on each interface
    """

    def __init__(self):
        self.system_state = "INIT STRING"
        self.previous_system_state = self.system_state
        self.button_state = BtnState()
        self.previous_button_state = BtnState()

        self.trial_number = 0
        self.participant_id = 'NULL'
        self.interface_type = 'NULL'
        self.trial_type = "NULL"
        self.run_number = 0
        self.new_trial = False
        self.new_run = False
        self.write_busy = False

        self.participant_id_sub = rospy.Subscriber("participant_id", String, self.participant_id_cb)
        self.trial_number_sub = rospy.Subscriber("trial_number", UInt64, self.trial_number_cb)
        self.trial_type_sub = rospy.Subscriber("trial_type", String, self.trial_type_cb)
        self.interface_type_sub = rospy.Subscriber("interface_type", String, self.interface_type_cb)

        self.system_state_sub = rospy.Subscriber("logging_topic", String, self.logging_topic_cb)

        rospy.init_node('study_system_logger', anonymous=True)

    def participant_id_cb(self, data):
        self.participant_id = data.data

    def trial_type_cb(self, data):
        self.trial_type = data.data

    def interface_type_cb(self, data):
        self.interface_type = data.data

    def trial_number_cb(self, data):
        self.trial_number = data.data

    def logging_topic_cb(self, data):
        self.previous_system_state = self.system_state
        self.system_state = data.data
        self.log()

    def log(self):
        print("Current directory:{}".format(os.path.dirname(__file__)))

        filename = os.path.join(os.path.dirname(__file__), "study_logging/" + str(self.participant_id) + '_' + self.interface_type + '_' + self.trial_type + "_" + str(self.trial_number) + ".csv")
        print("Generating:{}".format(filename))
        if self.previous_system_state != self.system_state:
            f = open(filename, 'a')
            f.write(str(datetime.datetime.now()) + ',' + str(self.system_state) + '\n')
            f.close()


if __name__ == '__main__':
    logger = SuccessLogger()
    rospy.spin()
