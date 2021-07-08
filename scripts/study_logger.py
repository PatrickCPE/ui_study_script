#!/usr/bin/env python

# coding=utf-8
# Test Script for the GUI vs TUI study

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt64
import os
import random
import datetime


# If using python3 uncomment the raw_inputs and replace with input

class StudyManager:
    """
    Script to manage both logging of the Scooter's TUI and GUI, and to handle miscellaneous tasks like object pick order
    randomization and object confirmation (done via experimenters)
    """
    def __init__(self):
        self.num_trials = 24
        self.participant_id = 'NULL'
        self.interface_type = 'NULL'
        self.clutter_type = "NULL"
        self.object_opts = ["Apple", "Apple", "Banana", "Banana", "Cat", "Cat", "Dog", "Dog", "Echo", "Echo",
                            "Fox", "Fox", "Gyarados", "Gyarados", "Hello", "Hello", "Igloo", "Igloo", "Jack", "Jack",
                            "Kite", "Kite", "Lamb", "Lamb"]
        self.object_opts_copy = list(self.object_opts)
        self.run_number = 0
        self.trial_number = 0
        self.full_run = True
        self.trial_success_log = [-1] * self.num_trials
        self.run_type_pub = rospy.Publisher('full_run_flag', Bool, queue_size=10)
        self.success_pub = rospy.Publisher('run_success', Bool, queue_size=10)

        #Publishers for system_state_logger
        self.participant_id_pub = rospy.Publisher('participant_id', String, queue_size=10)
        self.trial_type_pub = rospy.Publisher('trial_type', String, queue_size=10)
        self.interface_type_pub = rospy.Publisher('interface_type', String, queue_size=10)
        self.trial_number_pub = rospy.Publisher('trial_number', UInt64, queue_size=10)

        rospy.init_node('study_manager', anonymous=True)

    def obtain_user_id(self):
        """
        Parses input to determine user state

        :return: None
        """
        valid = False
        while not valid:
            self.participant_id = raw_input("Please enter participant ID: ")
            # self.participant_id = input("Please enter participant ID: ")
            print("Please confirm this is the proper ID: {0}".format(self.participant_id))
            confirmation = raw_input("(Case Sensitive) y/n: ")
            # confirmation = input("(Case Sensitive) y/n: ")
            if confirmation == 'y':
                valid = True
        self.participant_id_pub.publish(self.participant_id)

    def obtain_interface_type(self):
        """
        Update clutter type to place into CSV file with log of this trial

        :return: None
        """
        type_valid = False
        while not type_valid:
            interface_name = raw_input("Please enter the name of the user interface(case sensitive) (gui/tui): ")
            # interface_name = input("Please enter the name of the user interface(case sensitive) (gui/tui): ")
            print("Please confirm this is the proper interface: {0}".format(interface_name))
            type_confirmation = raw_input("(Case Sensitive) y/n: ")
            # type_confirmation = input("(Case Sensitive) y/n: ")
            if type_confirmation == 'y':
                if interface_name == 'tui':
                    type_valid = True
                elif interface_name == 'gui':
                    type_valid = True
                else:
                    print("Invalid User Interface Name (case sensitive)")
        self.interface_type = interface_name

    def obtain_clutter_type(self):
        """
        Update clutter type to place into CSV file with log of this trial

        :return: None
        """
        clutter_valid = False
        while not clutter_valid:
            clutter_name = raw_input(
                "Please enter the name of the user interface(case sensitive) (uncluttered/cluttered): ")
            # clutter_name = input(
            #    "Please enter the name of the user interface(case sensitive) (uncluttered/cluttered): ")
            print("Please confirm this is the proper clutter type: {0}".format(clutter_name))
            clutter_confirmation = raw_input("(Case Sensitive) y/n: ")
            # clutter_confirmation = input("(Case Sensitive) y/n: ")
            if clutter_confirmation == 'y':
                if clutter_name == 'uncluttered':
                    clutter_valid = True
                elif clutter_name == 'cluttered':
                    clutter_valid = True
                    self.clutter_type = clutter_name
                else:
                    print("Invalid User Clutter Name:{} (case sensitive)".format(clutter_name))
        self.clutter_type = clutter_name

    def obtain_trial_type(self):
        """
        Parses input to determine trial type

        :return: None
        """
        self.obtain_interface_type()
        self.obtain_clutter_type()

    def create_csv(self):
        """
        Create CSV file with participant ID and log stored results into the file

        :return: None
        """
        filename = os.path.join(os.path.dirname(__file__), "study_logging/" + str(self.participant_id) + ".csv")
        print("Generating:{}".format(filename))
        f = open(filename, 'a')
        f.write(str(datetime.datetime.now()) + ',' + str(self.interface_type) + ',' + str(self.clutter_type) +'\n')
        range(0, self.num_trials, 1)
        for i in range(self.num_trials):
            f.write(str(datetime.datetime.now()) + ',' + str(self.trial_success_log[i]))
            f.write('\n')
        f.close()
        """
        filename = self.participant_id + ".csv"
        with open(filename, 'a') as csv_file:
            log_writer = csv.writer(csv_file, delimiter=',', quoting=csv.QUOTE_MINIMAL)
            log_writer.writerow((self.interface_type, self.clutter_type, self.trial_success_log))
        """


    def update_run_mode(self):
        """
        Update the system status to determine whether a full pick and place task is performed or if only a select is
        performed

        :return: None
        """
        self.full_run = False
        if not rospy.is_shutdown():
            if self.trial_number == 0:
                self.full_run = True
            elif self.trial_number == self.num_trials - 1:
                self.full_run = True
            self.run_type_pub.publish(self.full_run)

    def confirm_trial_success(self):
        """
        Parse user input and determine whether the tester determines the trial a success or a failure

        :return: None
        """
        trial_success_valid = False
        while not trial_success_valid:
            trial_success = raw_input("Enter t if the trial succeeded and f if it failed (case sensitive) (t/f): ")
            # trial_success = input("Enter t if the trial succeeded and f if it failed (case sensitive) (t/f): ")
            print("Please confirm your input: {0}".format(trial_success))
            input_confirmation = raw_input("(Case Sensitive) y/n: ")
            # input_confirmation = input("(Case Sensitive) y/n: ")
            if input_confirmation == 'y':
                if trial_success == 't':
                    trial_success_valid = True
                    trial_success = True
                elif trial_success == 'f':
                    trial_success_valid = True
                    trial_success = False
                else:
                    print("Invalid Response: {0} (case sensitive)".format(trial_success))
            self.success_pub.publish(trial_success)
            self.trial_success_log[self.trial_number] = trial_success

    def determine_grasp_object(self):
        """
        Randomizes from the list of grasp objects shown here except for the initial and final trial

        :return: None
        """
        if len(self.object_opts) > 0:
            choice = random.choice(self.object_opts)
            print("Pick up the {}".format(choice))
        self.object_opts.remove(choice)

    def start_trial(self):
        """
        Informs the tester which object to have the participant select(randomly chosen to avoid bias), and prompts the
        tester to confirm whether the task was a success or failure

        :return: None
        """
        self.obtain_trial_type()
        self.trial_type_pub.publish(self.clutter_type)
        self.interface_type_pub.publish(self.interface_type)

        print("Current Trial Scenario (Interface = {0})(Clutter = {1})".format(self.interface_type, self.clutter_type))
        while self.trial_number < self.num_trials:
            self.update_run_mode()  # TODO Make this work
            self.trial_number_pub.publish(self.trial_number)
            self.determine_grasp_object()
            print("Beginning trial {0}".format(self.trial_number))
            self.confirm_trial_success()
            self.trial_number = self.trial_number + 1
        self.trial_number = 0

        ui_study_manager.create_csv()

    def start_runs(self):
        while self.run_number < 4:
            self.start_trial()
            # Refresh the object list between trials
            self.object_opts = self.object_opts_copy
            self.run_number = self.run_number + 1


if __name__ == '__main__':
    print("Beginning User Interface Comparison")

    ui_study_manager = StudyManager()
    ui_study_manager.obtain_user_id()
    ui_study_manager.start_runs()
