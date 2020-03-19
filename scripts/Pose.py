#!/usr/bin/env python
import rospy

class Pose(object):
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation

    def __str__(self):
        return "the position is: " + str(self.position) + "the orientation is: " + str(self.orientation)

