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
import json
import urdf_parser_py.urdf
import kdl_parser_py.urdf
import PyKDL

class arm(object):
    """Arm class to define a logical arm on top of the servo controller.
    This allows to use a subset of servos to define different kinematic
    chains.

    The most practical use is to separate the servos used for the arm
    kinematic from the gripper itself.  The xArm is actually not a 6
    DOFs robot, it has 5 servos ([6, 5, 4, 3, 2]) for the arm and the
    last one ([1]).  As such it is underactuated.

    The typical use is:
    import lewansoul_xarm
    c = lewansoul_xarm.controller()
    arm = c.add_arm(6, 5, 4, 3, 2)
    gripper = c.add_arm(1)
    arm.home()
    position = arm.measured_jp()
    position[0] = position[0] + math.pi / 4.0
    arm.move_jp(position)
    """

    def __init__(self, controller, name, config_file, urdf_file = ''):
        """Initialize the robot using a controller and a list of servo ids.
        The constructor also creates a unique id for the arm using the
        controller's serial number and the list of servo ids used for
        the arm.
        """
        self._controller = controller
        self._name = name
        self._config_file = config_file
        self._configuration = {}
        self._servo_ids = []
        self._is_homed = False
        self._has_kin = False
        self.configure(config_file)
        if not urdf_file == '':
            self._has_kin = True
            self.configure_urdf(urdf_file)


    def __del__(self):
        self.disable()


    def is_homed(self):
        return self._is_homed


    def calibrate(self, reference_position_si = 0.0):
        """Calibrate the servos position offsets.  The xArm servos are
        absolute, i.e. you can turn off/on the controller and the
        origin will not be reset.  The issue is that "zero" can be
        anywhere depending on how you assembled he arm.  The calibrate
        method computes and offset between the reference positions
        ([0..0] by default) and the current position.

        The calibration procedure is:
        - disable the arm so the servos are free to move (arm.disable())
        - position the arm in "zero" position
        - call arm.calibrate()

        The position offsets are used after the calibration and saved
        in a JSON configuration file using the arm unique id (see
        __init__ documentation).  Future calls of home() or
        configure() will try to locate the JSON configuration file and
        load it.  This allows to perform the calibration to be a one
        time task.
        """
        # get raw positions from controller
        position_si = numpy.multiply(self._directions, self._controller.measured_jp(self._servo_ids))
        self._position_offsets = reference_position_si - position_si
        # save calibrations results for later use, convert array to list
        self._configuration['offsets'] = self._position_offsets.tolist()
        with open(self._config_file, 'w') as f:
            f.write(json.dumps(self._configuration, sort_keys = True,
                               indent = 4, separators = (', ', ': ')))

        # no need to home now
        self._is_homed = True


    def configure(self, config_file):
        """Configure the arm using a JSON configuration file.  The
        configuration file must contain the board ids, controller
        serial number and the position offsets vector (see
        calibrate()).
        """
        try:
            with open(config_file, 'r') as f:
                self._configuration = json.load(f)
        except ValueError as e:
            print(e)
            raise
        except:
            raise

        # serial number [required]
        if not 'controller-serial-number' in self._configuration.keys():
            raise Exception('controller-serial-number is not defined in ' + config_file)
        assert self._configuration['controller-serial-number'] == self._controller._serial_number

        # servo ids [required]
        if not 'servo-ids' in self._configuration.keys():
            raise Exception('servo-ids are not defined in ' + config_file)
        self._servo_ids = self._configuration['servo-ids']

        # offsets [optional]
        if not 'offsets' in self._configuration.keys():
            self._position_offsets = numpy.zeros(len(self._servo_ids))
        else:
            if len(self._configuration['offsets']) != len(self._servo_ids):
                raise Exception('incorrect number of offsets found in ' + config_file)
            else:
                self._position_offsets = numpy.array(self._configuration['offsets'])

        # directions [optional]
        if not 'directions' in self._configuration.keys():
            self._directions = numpy.ones(len(self._servo_ids))
        else:
            if len(self._configuration['directions']) != len(self._servo_ids):
                raise Exception('incorrect number of directions found in ' + config_file)
            else:
                self._directions = numpy.array(self._configuration['directions'])

        self._measured_jp = numpy.zeros(len(self._servo_ids))
        self._goal_jp =  numpy.zeros(len(self._servo_ids))
        self._measured_cp = PyKDL.Frame()


    def configure_urdf(self, urdf_file):
        self._kdl_urdf = urdf_parser_py.urdf.URDF.from_xml_file(urdf_file)
        status, self._kdl_tree = kdl_parser_py.urdf.treeFromUrdfModel(self._kdl_urdf)
        assert status
        self._kdl_chain = self._kdl_tree.getChain('base_link', 'link5')
        self._kdl_fk = PyKDL.ChainFkSolverPos_recursive(self._kdl_chain)
        self._kdl_joints = PyKDL.JntArray(5)



    def enable(self):
        """Enable the servo controllers.  This method first read the current
        positions and then asks the servos to "go to" the current
        position.
        """
        # afaik there's no command to just enable the arm
        position = self.measured_jp()
        self.move_jp(position)


    def disable(self):
        """Disable the servo controllers
        """
        self._controller.disable(self._servo_ids)


    def home(self):
        if not self._is_homed:
            self.get_data()
            self.move_jp(self._measured_jp)
            self._is_homed = True


    def get_data(self):
        self._measured_jp = numpy.multiply(self._directions, self._controller.measured_jp(self._servo_ids)) + self._position_offsets
        if self._has_kin:
            for i in range(5):
                self._kdl_joints[i] = self._measured_jp[i]
            status = self._kdl_fk.JntToCart(self._kdl_joints, self._measured_cp, 5)


    def measured_jp(self):
        return self._measured_jp


    def measured_cp(self):
        return self._measured_cp


    def goal_jp(self):
        """Return the last joint position goal.  The goal is based on the
        last move_jp call.
        """
        return self._goal_jp


    def move_jp(self, goals_si, time_s = 0.0):
        self._goal_jp = goals_si[:]
        self._controller.move_jp(self._servo_ids,
                                 numpy.multiply(self._directions, (goals_si - self._position_offsets)),
                                 time_s)
