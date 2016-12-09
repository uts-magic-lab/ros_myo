#!/usr/bin/env python

"""
Script that prints in bar plot
the value of the EMG sensors.

Needs ascii_graph:

sudo pip install ascii_graph

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""
from __future__ import print_function

from ascii_graph import Pyasciigraph
from random import randrange
from time import sleep
import os

import rospy
from ros_myo.msg import EmgArray


def test():
    while True:
        ea = EmgArray()
        arr = []
        for i in range(8):
            arr.append(randrange(0, 1024))
        ea.data = arr

        graph = Pyasciigraph(multivalue=False)
        emg_data = []
        for idx, d in enumerate(ea.data):
            emg_data.append((idx, d))
        emg_data.append(('MAX_VAL', 1024))
        for emg_line in graph.graph('EMG values:', emg_data):
            print(emg_line)

        sleep(0.5)
        os.system('clear')


if __name__ == '__main__':
    # test()
    rospy.init_node('emg_bar_plot')
    rospy.loginfo("Printing plot of EMG values...")

    def emg_cb(ea):
        # Clear screen
        print("\n\n\n\n\n\n\n\n\n\n\n\n")
        graph = Pyasciigraph()
        emg_data = []
        # This bar is just for scale
        emg_data.append(('MAX_VAL', 2048))
        for idx, d in enumerate(ea.data):
            bar_title = "EMG #" + str(idx)
            emg_data.append((bar_title, d))

        for emg_line in graph.graph('EMG values:', emg_data):
            print(emg_line)

    emg_sub = rospy.Subscriber('/myo_raw/myo_emg',
                               EmgArray,
                               emg_cb,
                               queue_size=1)

    rospy.loginfo("Awaiting publications...")
    rospy.spin()
