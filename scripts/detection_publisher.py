#!/usr/bin/python

import rospy
import argparse
from yaml import load
from os.path import expanduser
from turtlebot3_slam_3d.srv import GetObjectLocation, GetObjectLocationResponse
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose, Point

class DetectionPublisher(object):
    def __init__(self, filename):
        with open(filename, 'r') as infile:
            self.detections = load(infile)
        self.marker_id = 0
        self.rate = rospy.Rate(1)
        self.map_objects = self.make_marker_array(self.detections)
        self.map_publisher = rospy.Publisher('~map_objects', MarkerArray, queue_size=10)
        self.map_server = rospy.Service('~object_location', GetObjectLocation, self.object_location)
        self.process()

    def make_marker(self, name, x, y, z):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "basic_shapes"
        marker.type = 9 # text
        marker.action = Marker.ADD
        marker.id = self.marker_id
        self.marker_id += 1

        marker.text = name
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z+0.3 # Text appears slightly above the object

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.scale.z = 0.3

        return marker

    def make_marker_array(self, detections):
        markers = []
        for name, pos in detections.iteritems():
            for p in pos:
                marker = self.make_marker(name, *p)
                markers.append(marker)
        return MarkerArray(markers)

    def object_location(self, req):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()

        if self.detections.has_key(req.object_name):
            poses = []
            for point in self.detections[req.object_name]:
                pose = Pose()
                pose.position = Point(*point)
                poses.append(pose)
            pose_array.poses = poses
        return GetObjectLocationResponse(pose_array)

    def process(self):
        print "Starting detection publisher"
        while not rospy.is_shutdown():
            self.map_publisher.publish(self.map_objects)
            self.rate.sleep()

if __name__ == '__main__':
    # Argument Parser
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input_file', type=str, default='~/.ros/detections_dbscan.db')
    args, _ = parser.parse_known_args()

    # Init
    rospy.init_node('detection_publisher')
    DetectionPublisher(expanduser(args.input_file))
