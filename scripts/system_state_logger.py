#!/usr/bin/env python

# coding=utf-8
# Test Script for the GUI vs TUI study

import rospy
from std_msgs.msg import String, UInt64, Bool
from ur_scooter_msgs.msg import BtnState
from mongodb_logger import MongoDBLogger
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
        self.object_name = "NULL"
        self.trial_type = "NULL"
        self.run_number = 0
        self.new_trial = False
        self.new_run = False
        self.write_busy = False
        self.success = None

        self.participant_id_sub = rospy.Subscriber("participant_id", String, self.participant_id_cb)
        self.trial_number_sub = rospy.Subscriber("trial_number", UInt64, self.trial_number_cb)
        self.trial_type_sub = rospy.Subscriber("trial_type", String, self.trial_type_cb)
        self.interface_type_sub = rospy.Subscriber("interface_type", String, self.interface_type_cb)
        self.object_name_sub = rospy.Subscriber("object_name", String, self.object_name_cb)
        self.success_sub = rospy.Subscriber("success", Bool, self.success_cb)

        self.system_state_sub = rospy.Subscriber("logging_topic", String, self.logging_topic_cb)

        self.transitions = []
        self.mongodblogger = MongoDBLogger(pilot=True)

        rospy.init_node('study_system_logger', anonymous=True)

    def participant_id_cb(self, data):
        self.participant_id = data.data

    def trial_type_cb(self, data):
        self.trial_type = data.data

    def interface_type_cb(self, data):
        self.interface_type = data.data

    def trial_number_cb(self, data):
        self.trial_number = data.data

    def object_name_cb(self, data):
        self.object_name = data.data

    def logging_topic_cb(self, data):
        self.previous_system_state = self.system_state
        self.system_state = data.data
        if self.participant_id != "NULL":
            self.log()
            self.log_mongodb()

    def success_cb(self, data):
        self.success = data.data

    def log(self):
        print("Current directory:{}".format(os.path.dirname(__file__)))

        filename = os.path.join(os.path.dirname(__file__), "study_logging/{}_{}_{}_{}_{}.csv".format(
            self.participant_id,
            self.interface_type,
            self.trial_type,
            self.trial_number,
            self.object_name
        ))
        print("Generating:{}".format(filename))
        if self.previous_system_state != self.system_state:
            f = open(filename, 'a')
            f.write(str(datetime.datetime.now()) + ',' + str(self.system_state) + '\n')
            f.close()

    def log_mongodb(self):
        print("Adding transition")
        # keep array of state transitions
        current_state, next_state, button = self.system_state.replace(' ', '').split(',')
        self.transitions.append(
            {
                "timestamp": datetime.datetime.utcnow(),
                "button": button,
                "current_state": current_state,
                "next_state": next_state,
            }
        )

        # create shorthand for condition
        condition = self.interface_type[0] + self.trial_type[0]

        # if the next state is "drive_mode", log to MongoDB
        print("Next state: {}".format(next_state))
        if next_state == 'drive_mode' or next_state == "grasping":
            print("Adding entry to DB")
            self.mongodblogger.add_task_to_db(
                self.participant_id,
                self.object_name,
                condition,
                self.transitions,
                self.success
            )
            # reset transitions
            self.transitions = []


if __name__ == '__main__':
    logger = SuccessLogger()
    rospy.spin()
