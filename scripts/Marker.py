#!/usr/bin/env python
import rospy
from Locateble_object import Locateble_object
import numpy as np


class Marker(Locateble_object):
    def __init__(self, id, current_position, last_position=None):
        Locateble_object.__init__(self, id, current_position, last_position)
        self.movement = 0.0

    def calc_movement(self):
        diff = np.sqrt(((self.current_position.x - self.last_position.x)**2) + \
                       ((self.current_position.y - self.last_position.y)**2) + \
                       ((self.current_position.z - self.last_position.z)**2))
        return (diff)

    def __str__(self):
        return Locateble_object.__str__(self)

    #def __hash__(self):
     #   return hash(self.id)

    def __eq__(self, other):
        return self.id == other.id
