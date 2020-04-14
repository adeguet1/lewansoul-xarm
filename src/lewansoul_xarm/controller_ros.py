#  Author(s):  Anton Deguet
#  Created on: 2020-04

# (C) Copyright 2020 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import rospy
import lewansoul_xarm


class controller_ros(object):
    """Simple arm API wrapping around ROS messages
    """

    # initialize the controller
    def __init__(self, controller):
        self._controller = controller
        self._arms = []
        for arm in self._controller._arms:
            self._arms.append(lewansoul_xarm.arm_ros(arm))


    def loop(self, rate):
        ros_rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            for arm in self._arms:
                arm.publish()
            ros_rate.sleep()
