#!/usr/bin/env python

import rospy
import lewansoul_xarm

def shutdown():
    print('\nshutting down xArm...')
    global controller
    controller.shutdown()

def ros_main():
    # connect to xArm
    global controller # so we can shutdown
    controller = lewansoul_xarm.controller()
    arm = controller.add_arm("arm", "xArm-49770F673737-arm.json")
    arm.enable()
    arm.home()

    gripper = controller.add_arm("gripper", "xArm-49770F673737-gripper.json")
    gripper.enable()
    gripper.home()

    rospy.init_node('xArm_ros', anonymous = True)
    rospy.on_shutdown(shutdown)

    controller_ros = lewansoul_xarm.controller_ros(controller)
    controller_ros.loop(30) # 30 Hz seems to be the maximum the controller can perform


if __name__ == '__main__':
    try:
        ros_main()
    except rospy.ROSInterruptException:
        pass
