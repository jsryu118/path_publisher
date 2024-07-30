#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

def publish_path_client(service_name):
    rospy.wait_for_service(service_name)
    try:
        publish_path = rospy.ServiceProxy(service_name, Empty)
        publish_path()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('path_publisher_client')

    service_names = [
        'publish_path_/path1',
        'publish_path_/path2',
        'publish_path_/path3'
    ]

    for service in service_names:
        publish_path_client(service)
