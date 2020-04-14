#  Author(s):  Anton Deguet
#  Created on: 2020-04

# (C) Copyright 2020 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

"""

"""

import numpy
import rospy
import std_msgs.msg
import sensor_msgs.msg

class arm_ros(object):
    """
    """

    def __init__(self, arm):
        """
        """
        self._arm = arm

        ns = self._arm._name + '/'
        # create publishers
        self._measured_js_msg = sensor_msgs.msg.JointState()
        for i in range(len(self._arm._servo_ids)):
            self._measured_js_msg.name.append(self._arm._name + str(i + 1))
        self._measured_js_publisher = rospy.Publisher(ns + 'measured_js',
                                                      sensor_msgs.msg.JointState,
                                                      queue_size = 10)
        # create subscribers
        rospy.Subscriber(ns + "enable", std_msgs.msg.Empty, self.enable)
        rospy.Subscriber(ns + "disable", std_msgs.msg.Empty, self.disable)
        rospy.Subscriber(ns + "home", std_msgs.msg.Empty, self.home)
        rospy.Subscriber(ns + "calibrate", std_msgs.msg.Empty, self.calibrate)

    def publish(self):
        measured_jp = self._arm.measured_jp()
        self._measured_js_msg.position[:] = measured_jp.flat
        self._measured_js_publisher.publish(self._measured_js_msg)

    def enable(self, msg):
        self._arm.enable()

    def disable(self, msg):
        self._arm.disable()

    def home(self, msg):
        self._arm.home()

    def calibrate(self, msg):
        self._arm.calibrate()
