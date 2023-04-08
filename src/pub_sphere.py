#! /usr/bin/env python3

import rospy 
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np 

def sample_spherical(npoints, ndim=3):
    vec = np.random.randn(ndim, npoints)
    vec /= np.linalg.norm(vec, axis=0)
    return vec

if __name__ == '__main__':
    rospy.init_node("sphere_pub")

    # rospy.loginfo("hello world")

    pub = rospy.Publisher("/points", PointCloud2, queue_size=1)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            ]

    rate = rospy.Rate(10)

    xi, yi, zi = sample_spherical(100)
    points = np.stack([xi,yi,zi], axis=0).T
    points[:,0] += 5
    points[:,1] += -3
    # points[:,2] += 1
    # points = points.tolist() 
    # header = Header()
    # header.frame_id = "map"
    # msg = point_cloud2.create_cloud(header, fields, points)


    while not rospy.is_shutdown():
        if np.max(points[:,1] < 2):
            points[:,1] += 0.1
        points_list = points.tolist() 
        header = Header()
        header.frame_id = "map"
        msg = point_cloud2.create_cloud(header, fields, points_list)

        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        # rospy.loginfo("Published")
        rate.sleep()


    