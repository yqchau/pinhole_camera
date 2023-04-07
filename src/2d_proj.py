#! /usr/bin/env python3

import rospy 
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, Image 
from std_msgs.msg import Header

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np 
from cv_bridge import cv2_to_imgmsg

class PointProjector:

    def __init__(self):
        self.sub = rospy.Subscriber("/points", PointCloud2, callback=self.callback, queue_size=1)
        self.pub = rospy.Publisher("/image_points", PointCloud2, queue_size=1)
        self.image_pub = rospy.Publisher('/my_image_topic', Image, queue_size=1)
        # self.image_square_marker_pub = rospy.Publisher('/image_markers', Marker, queue_size=1)
        # self.arrow_pub = rospy.Publisher('/arrows', Marker, queue_size=1)

        self.marker_arr_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)

        self.F = 1.0
        self.n_pixel = 100


        marker = Marker()
        marker.id = 0
        marker.header.frame_id = 'image1'
        marker.type = Marker.CUBE
        marker.scale.x = 0.01
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 0.3
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.w = 1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0

        self.marker = marker
        self.rate = rospy.Rate(5)


    def callback(self, msg):
        rospy.loginfo("Received callback")
        image = np.zeros((self.n_pixel,self.n_pixel))

        points = point_cloud2.read_points(msg,field_names = ("x", "y", "z"), skip_nans=True)
        image_points = []

        marker_array = MarkerArray() 

        id = 1

        for p in points:
            x, y, z = p

            marker = Marker()
            marker.id = id 
            marker.header.frame_id = "camera"
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 0.3
            marker.pose.orientation.w = 1.0
            marker.points = [Point(0, 0, 0), Point(x,y,z)]
            
            # self.arrow_pub.publish(marker)

            x_image =  0
            y_image = y * self.F / x
            z_image = z * self.F / x

            # x_image, y_image, z_image = z_image, -1*x_image, -1*y_image

            image_points.append([x_image, y_image, z_image])

            u = int(self.n_pixel//2 - y_image * self.n_pixel) 
            v = int(self.n_pixel//2 - z_image * self.n_pixel)

            if (0<u<self.n_pixel and 0<v<self.n_pixel):
                image[v,u] = 255

                marker.color.r = 1
                marker.color.g = 1
                marker.color.b = 1
            
            marker_array.markers.append(marker)
            id += 1
        
        header = Header()
        header.frame_id = "image1"
        msg.header.stamp = rospy.Time.now()
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        ]
        msg = point_cloud2.create_cloud(header, fields, image_points)

        # image = np.stack([image]*3, axis=2)

        image = image.astype(np.uint8)
        msg_image = cv2_to_imgmsg(image, encoding='mono8')

        self.pub.publish(msg)
        self.image_pub.publish(msg_image)

        # self.marker.header.stamp = rospy.Time.now()
        # self.image_square_marker_pub.publish(self.marker)
        marker_array.markers.append(self.marker)

        self.marker_arr_pub.publish(marker_array)

        

        rospy.loginfo("Published")

        self.rate.sleep()

if __name__ == '__main__':

    rospy.init_node("sphere_sub")

    proj = PointProjector()

    rospy.spin()
