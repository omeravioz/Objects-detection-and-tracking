#!/usr/bin/env python
import rospy
import numpy


class Locateble_object(object):
    def __init__(self, id=0, current_position=None, last_position=None):
        self.id = id
        self.current_position = current_position
        self.last_position = last_position


    def __str__(self):
        return str(self.id) + str(self.current_position)


