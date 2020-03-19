#!/usr/bin/env python
import rospy

class Orientation(object):
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __str__(self):
        return "x " + str(self.x) + " y " + str(self.y) + " z " + str(self.z) + " w " + str(self.w)