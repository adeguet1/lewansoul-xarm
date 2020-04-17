#!/usr/bin/env python

import os
import rospy
import rospkg
import lewansoul_xarm

def shutdown():
    print('\nshutting down xArm...')
    global controller
    controller.shutdown()

def ros_main():
    # connect to xArm
    global controller # so we can shutdown
    controller = lewansoul_xarm.controller()

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('lewansoul_xarm')
    
    urdf_file = pkg_path + '/urdf/xArm.urdf'
    if not os.path.isfile(urdf_file):
        print('couldn\'t find file: ' + urdf_file + '. cartesian moves will be disabled')
        urdf_file = ''
    arm = controller.add_arm("arm", "xArm-49770F673737-arm.json", urdf_file)
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
