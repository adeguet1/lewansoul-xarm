#  Author(s):  Anton Deguet
#  Created on: 2020-04

# (C) Copyright 2020 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

"""

Code based on:
https://gist.github.com/maximecb/7fd42439e8a28b9a74a4f7db68281071
https://github.com/maximkulkin/lewansoul-lx16a

sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0 python-hid
pip install --user easyhid
"""

import math
import easyhid
import numpy
import PyKDL
import itertools
import lewansoul_xarm

def integer_to_bits(v):
    lsb = v & 0xFF
    msb = v >> 8
    return lsb, msb

class controller(object):
    """Simple arm API wrapping around ROS messages
    """

    # initialize the controller
    def __init__(self, pid = 22352):
        # base class constructor in separate method so it can be called in derived classes
        self.__init_controller(pid)


    def __init_controller(self, pid = 22352):
        """
        """
        # stores an enumeration of all the connected USB HID devices
        en = easyhid.Enumeration()

        # return a list of devices based on the search parameters
        devices = en.find(vid = 1155, pid = pid)

        # print a description of the devices found
        for dev in devices:
            print(dev.description())

        assert len(devices) > 0
        self._controller = devices[0]

        # open a device
        self._controller.open()
        self._serial_number = self._controller.serial_number.encode('ascii')
        print('connected to xArm controller serial number: ' + self._serial_number)

        # compute bits_to_rads: Lewansoul bits are 1000 for 240 degree
        # range, then convert from degrees to radians
        self.bits_to_si = math.radians((240.0 / 1000.0))


    def __del__(self):
        print('closing xArm controller')
        self._controller.close()


    def add_arm(self, *servo_ids):
        new_arm = lewansoul_xarm.arm(self, servo_ids)
        return new_arm


    def measured_jp(self, servo_ids):
        """
        Read the position of all servos
        ServoPositionRead 21 (byte)count { (byte)id }; (byte)count { (byte)id (ushort)position }
        """
        nb_servos = len(servo_ids)
        self._controller.write(itertools.chain([0x55, 0x55,
                                                3 + nb_servos, # length of command
                                                21, # command id
                                                nb_servos]
                                               ,
                                               servo_ids))
        response = self._controller.read()
        # check first two elements of answer
        assert response[0] == 0x55
        assert response[1] == 0x55
        # response should march number of servos
        count = response[4]
        assert count == nb_servos
        # array used to store SI positions
        positions = numpy.zeros(nb_servos)
        for i in range(nb_servos):
            # check that servo ids match
            id = response[5 + 3 * i]
            assert id == servo_ids[i]
            # compute joint position
            p_lsb = response[5 + 3 * i + 1] # least significant bit
            p_msb = response[5 + 3 * i + 2] # most significant bit
            pos_bits = (p_msb << 8) + p_lsb
            positions[i] = pos_bits * self.bits_to_si
        return positions


    def move_jp(self, servo_ids, goals_si, time_s = 0.0):
        assert len(goals_si) == len(servo_ids)

        # should be replaced by computation using optimal joint velocities
        if time_s == 0.0:
            time_s = 5.0
        elif time_s > 20:
            time_s = 20.0
        time_ms = int(time_s * 1000);

        # convert
        goals = goals_si / self.bits_to_si

        # command
        command = [0x55, 0x55,
                   len(5 + goals * 3), # msg length
                   3, # cmd
                   len(goals)]
        command.extend(integer_to_bits(time_ms))
        for i, servo_id in enumerate(servo_ids):
            command.append(servo_id)
            command.extend(integer_to_bits(int(goals[i])))

        self._controller.write(command)
