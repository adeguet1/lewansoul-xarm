#!/usr/bin/env python

import rospy

import lewansoul_xarm

def ros_main():
    # connect to xArm
    controller = lewansoul_xarm.controller()
    arm = controller.add_arm("xArm", 6, 5, 4, 3, 2) # only the first 5 joints are for kinematics
    arm.enable()
    arm.home()

    gripper = controller.add_arm("gripper", 1)
    gripper.enable()
    gripper.home()

    rospy.init_node('xArm_ros', anonymous=True)

    controller_ros = lewansoul_xarm.controller_ros(controller)
    controller_ros.loop(30) # 30 Hz seems to be the maximum the controller can perform


if __name__ == '__main__':
    try:
        ros_main()
    except rospy.ROSInterruptException:
        pass
