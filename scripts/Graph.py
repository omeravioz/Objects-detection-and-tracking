#!/usr/bin/env python
import rospy
import setting as s
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import UInt32MultiArray
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from aruco_msgs.msg import MarkerArray
from Position import Position
from Orientation import Orientation
from Pose import Pose
from Marker import Marker
from Object import Object
from Camera import Camera
from Locateble_object import Locateble_object
from timeit import default_timer as timer
import collections
import csv
import sys
import datetime



class Graph(object):
    def __init__(self):
        self.objects = []
        self.outgoing = {}
        self.reference_markers = [173]
        self.shown_reference_markers = []
#       self.location = []

def callback_markers(data):
    #rm = set_reference_marker(data)
    rm = [Position(0.0, 0.0, 0.0),0]
    if rm is not None and rm[1] < 0.003:
        s.frame_markers = set_frame_markers(data, rm)
        set_Graph()

        if s.counter == 2:
            obj_size = len(Graph.objects)
            start = timer()
            frame_objects = set_frame_objects()
            set_objects(frame_objects)
            if obj_size != len(Graph.objects):
                runtime = (timer()-start-s.subtract_time)*100
                with open('object_data ' + s.now.strftime("%Y-%m-%d time %H-%M ") + '.csv', mode='a') as data:
                    s.objects_writer = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    s.objects_writer.writerow(['set object run time', runtime])
                    s.objects_writer.writerow(['memory of objects', sys.getsizeof(Graph.objects)])
                    s.objects_writer.writerow([])


            print("###########################################")

    if s.counter == 2:
        s.counter = 0
    s.counter += 1


def set_reference_marker(data):
    for fm in data.markers:
        if fm.id in Graph.reference_markers:

            for s in Graph.shown_reference_markers:
                if fm.id == s.id:
                    position = fm.pose.pose.position
                    s.last_position = s.current_position
                    s.current_position = Position(position.x, position.y, position.z)
                    return [s, s.calc_movement()]


            position = fm.pose.pose.position
            Graph.shown_reference_markers.append(Marker(fm.id, Position(position.x, position.y, position.z)))
            return None

    return None


def find_markers():
    rospy.init_node('Marker', anonymous=True)
    rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, callback_markers)
    rospy.spin()


def set_frame_markers(data, rm):
    s.frame_markers = []
    for m in data.markers:
        stop = False
        for srm in Graph.shown_reference_markers:
            if m.id == srm.id:
                stop = True
                break
        if stop is False:
            marker_found = False
            marker_id = m.id
            position = m.pose.pose.position
            #marker_position = Position((position.x-rm[0].current_position.x),
                                       #(position.y-rm[0].current_position.y),
                                       #(position.z-rm[0].current_position.z))
            marker_position = Position(position.x, position.y, position.z)
            marker_found = False

            for out in Graph.outgoing.keys():
                if m.id == out.id:
                    marker_found = True
                    if s.counter == 2:        # update location once in 30 iterations
                        out.last_position = out.current_position
                        out.current_position = marker_position
                        s.frame_markers.append(out)

            if not marker_found:
                marker = Marker(marker_id, marker_position)
                s.frame_markers.append(marker)
                Graph.outgoing[marker] = set()

    return s.frame_markers

def set_Graph():
    start_time = timer()
    for fm1 in s.frame_markers:
        for fm2 in s.frame_markers:
            if fm1 != fm2:
                Graph.outgoing[fm1].add(fm2)
    runtime = (timer()-start_time)*100

    if len(Graph.outgoing) > s.graph_size:
        with open('graph_data ' + s.now.strftime("%Y-%m-%d time %H-%M ") + '.csv', mode='a') as data:
            s.objects_writer = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            s.objects_writer.writerow(['Markers graph'])
            s.objects_writer.writerow(['number of markers', len(Graph.outgoing)])
            s.objects_writer.writerow(['memory of graph', sys.getsizeof(Graph.outgoing)])
            s.objects_writer.writerow(['run time', runtime])
        s.graph_size = len(Graph.outgoing)

def set_objects(frame_objects):
    s.subtract_time = 0
    objects_to_add = []
    for ind in range(len(frame_objects)):
        flag = True
        is_move = is_object_move(frame_objects[ind])

        o = 0
        # for o in Graph.objects:
        while o < len(Graph.objects):
            if set(frame_objects[ind]).issubset(Graph.objects[o].markers):  # object contain frame_object
                flag = False

                if len(frame_objects[ind]) != len(Graph.objects[o].markers) and not\
                        Graph.objects[o].has_moved and is_move:  # this object is already exist
                    Graph.objects[o].markers = [x for x in Graph.objects[o].markers if x not in frame_objects[ind]]
                    newobject = Object(is_move, frame_objects[ind])
                    objects_to_add.append(newobject)
                    start = timer()
                    write_object_distance(newobject,frame_objects)
                    s.subtract_time += (timer() - start)

                elif len(frame_objects[ind]) == len(Graph.objects[o].markers):
                    Graph.objects[o].has_moved = Graph.objects[o].has_moved or is_move

                break
            elif set(Graph.objects[o].markers).issubset(frame_objects[ind]):  # frame_object contain object
                if is_move:
                    if object_already_exist(frame_objects[ind]):
                        Graph.objects.pop(o)
                        o = o - 1
                    else:
                        Graph.objects[o].markers = frame_objects[ind]
                        Graph.objects[o].has_moved = True
                else:
                    frame_objects[ind] = [x for x in frame_objects[ind] if x not in Graph.objects[o].markers]
                flag = False

            elif is_object_contain_markers(o, frame_objects[ind]):
                flag = False
                break

            o = o + 1

        if flag:
            newobject = Object(is_move, frame_objects[ind])
            objects_to_add.append(newobject)
            start = timer()
            write_object_distance(newobject, frame_objects)
            s.subtract_time += (timer() - start)

    for ota in objects_to_add:
        Graph.objects.append(ota)

    for i in Graph.objects:
        print("---------------------------------------")
        print(i.has_moved)
        for j in i.markers:
            print(j.id)

def is_object_move(markers):
    for m in markers:
        print(m.calc_movement())
        if m.calc_movement() < 0.0007:
            return False
    return True

def is_object_contain_markers(o, markers):
    for m in markers:
        if set([m]).issubset(Graph.objects[o].markers):
            return True

    return False

def set_frame_objects():
    objects = []
    can_skip = []
    counter = -1
    for i in range(len(s.frame_markers)):
        if can_skip.__contains__(i):
            continue
        flag = True

        objects.append([s.frame_markers[i]])
        counter += 1

        for j in range(i + 1, len(s.frame_markers)):
            if same_object(s.frame_markers[i], s.frame_markers[j]):
                objects[counter].append(s.frame_markers[j])
                can_skip.append(j)
                print("%d and %d totally the same objectttttt" % (s.frame_markers[i].id, s.frame_markers[j].id))

            else:
                flag = False
                print("%d and %d is not the same object" % (s.frame_markers[i].id, s.frame_markers[j].id))
            print("----------------")
        if flag:
            break
    '''for i in objects:
        print("-------------------")
        for j in i:
            print("marker %d ," % j.id)
    print("#########################")'''

    return objects

def same_object(marker1, marker2):

    if marker1.last_position is not None and marker2.last_position is not None:
        pair = [marker1, marker2]
        if marker2.id < marker1.id:
            pair.reverse()

        dist = distance_between_markers(pair)
        dist_diff = abs(dist[0]-dist[1])

        print(dist_diff)
        if dist_diff > 0.0004:
            return False

        return True

    else:
        print("no last position")
        return False

def write_object_distance(object, frame_objects):
    oc1 = calculate_object_center(object)
    with open('object_data ' + s.now.strftime("%Y-%m-%d time %H-%M ") + '.csv', 'a') as data:
        s.objects_writer = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        s.objects_writer.writerow(['object ', object])
        s.objects_writer.writerow(['time ', round(timer()-s.start, 3)])
        s.objects_writer.writerow(['markers on frame ', len(s.frame_markers)])
        s.objects_writer.writerow(['markers on system ', len(Graph.outgoing)])
        s.objects_writer.writerow(['time to recognize ', 0])
        s.objects_writer.writerow(['distance between objects '])
        for o in Graph.objects:
            if o != object:
                for fo in frame_objects:
                    if collections.Counter(fo) == collections.Counter(o.markers):
                        oc2 = calculate_object_center(o)
                        location_diff = [oc1[0]-oc2[0], oc1[1]-oc2[1], oc1[2]-oc2[2]]
                        location_diff = np.array(location_diff)
                        dist = np.sum(location_diff ** 2, axis=0)
                        fdist = np.sqrt(dist)
                        s.objects_writer.writerow([o, fdist*100])


def distance_between_markers(pair):
    min_curr_position = pair[0].current_position
    max_curr_position = pair[1].current_position
    min_last_position = pair[0].last_position
    max_last_position = pair[1].last_position

    curr_relative_pos = [min_curr_position.x - max_curr_position.x,
                         min_curr_position.y - max_curr_position.y,
                         min_curr_position.z - max_curr_position.z]

    last_relative_pos = [min_last_position.x - max_last_position.x,
                         min_last_position.y - max_last_position.y,
                         min_last_position.z - max_last_position.z]

    p1 = np.array(curr_relative_pos)
    p2 = np.array(last_relative_pos)

    curr_dist = np.sum(p1 ** 2, axis=0)
    curr_dist = np.sqrt(curr_dist)
    last_dist = np.sum(p2 ** 2, axis=0)
    last_dist = np.sqrt(last_dist)
    return [curr_dist, last_dist]

def calculate_object_center(object):
    object_center = [0, 0, 0]
    object_av = [0, 0, 0]
    for m in object.markers:
        object_center[0] += m.current_position.x
        object_center[1] += m.current_position.y
        object_center[2] += m.current_position.z
    for oc in range(len(object_center)):
        object_av[oc] = (object_center[oc]/len(object.markers))

    return object_av

def object_already_exist(obj):
    for i in Graph.objects:
        if collections.Counter(i.markers) == collections.Counter(obj):
            return False
    return True

def save_data():
    with open('object_data ' + s.now.strftime("%Y-%m-%d time %H-%M ") + '.csv', mode='a') as data:
        counter = 1
        s.objects_writer = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for i in Graph.objects:
            s.objects_writer.writerow(['Object' + str(i)])
            s.objects_writer.writerow(['number of Markers: ', len(i.markers)])
            s.objects_writer.writerow(['object location ', calculate_object_center(i)])
            s.objects_writer.writerow(['distance between markers'])
            for j in range(len(i.markers)):
                for k in range(j+1, len(i.markers)):
                    #dist = distance_between_markers([i.markers[j], i.markers[k]])
                    s.objects_writer.writerow(['marker %d' % i.markers[j].id, 'marker %d' % i.markers[k].id,0])


if __name__ == '__main__':
    s.now = datetime.datetime.now()
    with open('graph_data ' + s.now.strftime("%Y-%m-%d time %H-%M ") + '.csv', 'w') as data:
        s.objects_writer = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    with open('object_data ' + s.now.strftime("%Y-%m-%d time %H-%M ") + '.csv', 'w') as data:
        s.objects_writer = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    s.graph_size = 0
    s.start = timer()
    s.first_run = False
    s.last_markers_frame = []
    s.counter = 0
    Graph = Graph()
    find_markers()
    save_data()














