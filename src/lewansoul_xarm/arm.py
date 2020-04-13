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

class arm(object):

    def __init__(self, controller, servo_ids):
        self._controller = controller
        self._servo_ids = servo_ids


    def measured_jp(self):
        """
        Read the position of all 6 servos
        ServoPositionRead 21 (byte)count { (byte)id }; (byte)count { (byte)id (ushort)position }
        """
        return self._controller.measured_jp(self._servo_ids)

    def move_jp(self, goals_si, time_s = 0.0):
        self._controller.move_jp(self._servo_ids, goals_si, time_s)
