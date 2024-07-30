#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_srvs.srv import Empty, EmptyResponse

class PathPublisherService:
    def __init__(self, filename, topic_name, frame_id):
        self.filename = filename
        self.topic_name = topic_name
        self.path_pub = rospy.Publisher(topic_name, Path, queue_size=10, latch=True)
        self.frame_id = frame_id
        self.path = self.read_path_from_file()
        self.service = rospy.Service(f'publish_path_{topic_name}', Empty, self.handle_publish_path)

    def read_path_from_file(self):
        try:
            data = np.loadtxt(self.filename)
            x_vals = data[:, 0]
            y_vals = data[:, 1]
            z_vals = data[:, 2]

            path = Path()
            path.header.frame_id = self.frame_id

            for x, y, z in zip(x_vals, y_vals, z_vals):
                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.w = 1.0  # Identity quaternion

                path.poses.append(pose)

            return path
        except Exception as e:
            rospy.logerr(f"Error reading path from file {self.filename}: {e}")
            return Path()

    def handle_publish_path(self, req):
        self.path.header.stamp = rospy.Time.now()
        for pose in self.path.poses:
            pose.header.stamp = self.path.header.stamp
        self.path_pub.publish(self.path)
        return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('path_publisher_service', anonymous=True)

    frame_id = rospy.get_param('~frame_id', 'map')

    filename1 = rospy.get_param('~filename1', '/default/path/to/111.txt')
    topic_name1 = rospy.get_param('~topic_name1', '/path1')
    PathPublisherService(filename1, topic_name1, frame_id)

    filename2 = rospy.get_param('~filename2', '/default/path/to/222.txt')
    topic_name2 = rospy.get_param('~topic_name2', '/path2')
    PathPublisherService(filename2, topic_name2, frame_id)

    filename3 = rospy.get_param('~filename3', '/default/path/to/333.txt')
    topic_name3 = rospy.get_param('~topic_name3', '/path3')
    PathPublisherService(filename3, topic_name3, frame_id)

    rospy.spin()
