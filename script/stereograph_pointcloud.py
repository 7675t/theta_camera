#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

# THEATAの画像を取得，球体のPointCloudとして出力する
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2

from std_msgs.msg import Header

def create_cloud_xyz32rgb(header, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message with 3 float32 fields (x, y, z).
    
    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param points: The point cloud points.
    @type  points: iterable
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 12, PointField.UINT32, 1)]
    return pcl2.create_cloud(header, fields, points)    

class point_cloud_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.center_x = rospy.get_param("center_x", 310)
        self.center_y = rospy.get_param("center_y", 320)
        self.radius = rospy.get_param("radius", 285)
        self.sphere_radius = 1.0
        self.step = 2
        
        self.face_sub = rospy.Subscriber("face/image_raw", Image, self.callback_face)
        self.rear_sub = rospy.Subscriber("rear/image_raw", Image, self.callback_rear)
        self.image_pub = rospy.Publisher("image_out", Image, queue_size=1)
        self.face_point_pub = rospy.Publisher("face_points", PointCloud2, queue_size=1)
        self.rear_point_pub = rospy.Publisher("rear_points", PointCloud2, queue_size=1)

    def callback_face(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape

        # cv2.circle(cv_image, (self.center_x, self.center_y), self.radius, (0, 255, 0))

        # 単位球面上での等分の離散化(theta, phi)
        sphere_image = np.ndarray([int(90 / self.step)+1, int(360 / self.step)+1, channels], dtype=np.uint8)

        # 点群もついでに作成
        points = []
        for theta_i in range(int(90 / self.step) + 1):
            for phi_i in range(int(360 / self.step) + 1):
                theta = self.step * theta_i * np.pi / 180
                phi = self.step * phi_i * np.pi / 180
                # 単位球面上の３次元座標(xs, ys, zs)
                xs = np.sin(theta)*np.cos(phi)
                ys = np.sin(theta)*np.sin(phi)
                zs = np.cos(theta)
                # 立体射影(THETA)上の座標(xt, yt)
                xt = self.center_x + self.radius * xs / (1 + zs)
                yt = self.center_y + self.radius * ys / (1 + zs)
                try:
                    color = cv_image[int(xt), int(yt)]
                except:
                    print(xs, ys, zs)
                    print(int(xt), int(yt))
                # 球面に写しとる
                sphere_image[theta_i, phi_i] = color
                icolor = int((color[2] << 16) | (color[1] << 8) | color[0])
                points.append([self.sphere_radius * xs, self.sphere_radius * ys, self.sphere_radius * zs, int(icolor)])

        #create pcl from points
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'face_frame'
        cloud = create_cloud_xyz32rgb(header, points)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(sphere_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

        self.face_point_pub.publish(cloud)

    def callback_rear(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape

        # cv2.circle(cv_image, (self.center_x, self.center_y), self.radius, (0, 255, 0))

        # 単位球面上での等分の離散化(theta, phi)
        sphere_image = np.ndarray([int(90 / self.step)+1, int(360 / self.step)+1, channels], dtype=np.uint8)

        # 点群もついでに作成
        points = []
        for theta_i in range(int(90 / self.step) + 1):
            for phi_i in range(int(360 / self.step) + 1):
                theta = self.step * theta_i * np.pi / 180
                phi = self.step * phi_i * np.pi / 180
                # 単位球面上の３次元座標(xs, ys, zs)
                xs = np.sin(theta)*np.cos(phi)
                ys = np.sin(theta)*np.sin(phi)
                zs = np.cos(theta)
                # 立体射影(THETA)上の座標(xt, yt)
                xt = self.center_x + self.radius * xs / (1 + zs)
                yt = self.center_y + self.radius * ys / (1 + zs)
                try:
                    color = cv_image[int(xt), int(yt)]
                except:
                    print(xs, ys, zs)
                    print(int(xt), int(yt))
                # 球面に写しとる
                sphere_image[theta_i, phi_i] = color
                icolor = int((color[2] << 16) | (color[1] << 8) | color[0])
                points.append([self.sphere_radius * xs, self.sphere_radius * ys, self.sphere_radius * zs, int(icolor)])
                
        #create pcl from points
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'rear_frame'
        cloud = create_cloud_xyz32rgb(header, points)

        self.rear_point_pub.publish(cloud)

def main(args):
    pc = point_cloud_converter()
    rospy.init_node('point_cloud_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


