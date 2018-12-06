#!/usr/bin/python

import rospy
from yaml import dump
from geometry_msgs.msg import PoseArray, Pose
from detection_clustering import DetectionClustering

class DetectionCollector(object):
    def __init__(self):
        self.detected = {}
        self.detection_names = rospy.get_param('/darknet_ros/yolo_model/detection_classes/names')
        rospy.Subscriber('/cluster_decomposer/centroid_pose_array', PoseArray, self.collect)
        print 'Searching for objects...'

    def __del__(self):
        print "Writing to detections_raw.db..."
        with open('detections_raw.db', 'w') as outfile:
            dump(self.detected, outfile)

        print "Writing to detections_dbscan.db..."
        dc = DetectionClustering(self.detected)
        with open('detections_dbscan.db', 'w') as outfile:
            dump(dc.clusters, outfile)

    def update_key(self, key, val):
        if self.detected.has_key(key):
            self.detected[key].append(val)
        else:
            self.detected[key] = [val]

    def collect(self, msg):
        for i, pose in enumerate(msg.poses):
            if pose != Pose():
                pos = pose.position
                val = [pos.x, pos.y, pos.z]
                key = self.detection_names[i]
                print 'Found a {} at {}'.format(key, val)
                self.update_key(key, val)

if __name__ == '__main__':
    rospy.init_node('detection_collector')
    DetectionCollector()
    rospy.spin()
