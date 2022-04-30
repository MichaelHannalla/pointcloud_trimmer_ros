#! /usr/bin/env python

import rospy
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2

def pointcloud_clbk(untrimmed_cloud_msg):
    untrimmed_cloud_np  = ros_numpy.numpify(untrimmed_cloud_msg)
    points = np.array([untrimmed_cloud_np['x'], (untrimmed_cloud_np['y']) ,
        (untrimmed_cloud_np['z'])]).copy().transpose()

    x_in_range = np.logical_and(points[:,0] > min_x, points[:,0] < max_x)
    y_in_range = np.logical_and(points[:,1] > min_y, points[:,1] < max_y)
    z_in_range = np.logical_and(points[:,2] > min_z, points[:,2] < max_z)
    
    xy_in_range = np.logical_and(x_in_range, y_in_range)
    in_range = np.logical_and(xy_in_range, z_in_range)
    points_trimmed = points[in_range]

    points_dict = np.zeros(len(points_trimmed), dtype=[('x', 'f4'), ('y', 'f4'),('z', 'f4')])
    points_dict['x'] = points_trimmed[:,0]
    points_dict['y'] = points_trimmed[:,1]
    points_dict['z'] = points_trimmed[:,2]

    trimmed_cloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(points_dict)
    trimmed_cloud_msg.header.frame_id = untrimmed_cloud_msg.header.frame_id
    trimmed_cloud_msg.header.stamp = rospy.Time.now()

    trimmed_cloud_pub.publish(trimmed_cloud_msg)
    

if __name__ == "__main__":

    rospy.init_node("pointcloud_trimmer_ros_node")
    rospy.logwarn("Starting pointcloud trimming node.")

    # Get the trimming parameters
    min_x = rospy.get_param("pointcloud_trimmer_node/min_x", default=-24.0)
    max_x = rospy.get_param("pointcloud_trimmer_node/max_x", default=24.0)
    min_y = rospy.get_param("pointcloud_trimmer_node/min_y", default=-24.0)
    max_y = rospy.get_param("pointcloud_trimmer_node/max_y", default=24.0)
    min_z = rospy.get_param("pointcloud_trimmer_node/min_z", default=0.05)
    max_z = rospy.get_param("pointcloud_trimmer_node/max_z", default=0.8)

    # Get pointcloud names
    cloud_in = rospy.get_param("pointcloud_trimmer_node/pointcloud_in", default="os1_cloud_node/points")
    cloud_out = rospy.get_param("pointcloud_trimmer_node/pointcloud_out", default="os1_cloud_node/points_filtered")

    # Setup ROS publishers and subscribers
    rospy.Subscriber(cloud_in, PointCloud2, callback=pointcloud_clbk)
    trimmed_cloud_pub = rospy.Publisher(cloud_out, PointCloud2, queue_size= 10)

    rospy.spin()
