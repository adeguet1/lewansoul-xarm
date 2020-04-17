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
import geometry_msgs.msg

class arm_ros(object):
    """
    """

    def __init__(self, arm):
        """
        """
        self._arm = arm

        ns = self._arm._name + '/'
        # create publishers and corresponding message
        # joint space
        self._js_msg = sensor_msgs.msg.JointState()
        for i in range(len(self._arm._servo_ids)):
            self._js_msg.name.append(self._arm._name + str(i + 1))
        self._measured_js_publisher = rospy.Publisher(ns + 'measured_js',
                                                      sensor_msgs.msg.JointState,
                                                      queue_size = 10)
        self._goal_js_publisher = rospy.Publisher(ns + 'goal_js',
                                                  sensor_msgs.msg.JointState,
                                                  queue_size = 10)
        # cartesian space
        if self._arm._has_kin:
            self._cp_msg = geometry_msgs.msg.TransformStamped()
            self._measured_cp_publisher = rospy.Publisher(ns + 'measured_cp',
                                                          geometry_msgs.msg.TransformStamped,
                                                          queue_size = 10)

        # create subscribers
        rospy.Subscriber(ns + "enable", std_msgs.msg.Empty, self.enable)
        rospy.Subscriber(ns + "disable", std_msgs.msg.Empty, self.disable)
        rospy.Subscriber(ns + "home", std_msgs.msg.Empty, self.home)
        rospy.Subscriber(ns + "calibrate", std_msgs.msg.Empty, self.calibrate)
        rospy.Subscriber(ns + "move_jp", sensor_msgs.msg.JointState, self.move_jp)

    def publish(self):
        try:
            # update data from controller
            self._arm.get_data()
            # measured joint state
            measured_jp = self._arm.measured_jp()
            self._js_msg.position[:] = measured_jp.flat
            self._measured_js_publisher.publish(self._js_msg)
            # goal joint position
            goal_jp = self._arm.goal_jp()
            self._js_msg.position[:] = goal_jp.flat
            self._goal_js_publisher.publish(self._js_msg)
            # measured cartesian position
            if self._arm._has_kin:
                measured_cp = self._arm.measured_cp()
                t = self._cp_msg.transform
                t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = measured_cp.M.GetQuaternion()
                t.translation.x = measured_cp.p[0]
                t.translation.y = measured_cp.p[1]
                t.translation.z = measured_cp.p[2]
                self._measured_cp_publisher.publish(self._cp_msg)
        except:
            print('failed to communicate with HID device, arm might be shutting down or controller has been turned off')
            rospy.signal_shutdown('HID communication error')

    def enable(self, msg):
        self._arm.enable()

    def disable(self, msg):
        self._arm.disable()

    def home(self, msg):
        self._arm.home()

    def calibrate(self, msg):
        self._arm.calibrate()

    def move_jp(self, msg):
        goal = numpy.array(msg.position)
        self._arm.move_jp(goal)
