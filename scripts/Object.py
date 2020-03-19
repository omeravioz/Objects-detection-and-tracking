#!/usr/bin/env python
import rospy


class Object(object):

    def __init__(self, has_moved, markers=[]):
        self.markers = markers
        self.has_moved = has_moved

    def __str__(self):
        obj = []
        for m in self.markers:
            obj.append(m.id)
        return str(obj)
