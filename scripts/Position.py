#!/usr/bin/env python
import rospy


class Position(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "x " + str(self.x) + " y " + str(self.y) + " z " + str(self.z)