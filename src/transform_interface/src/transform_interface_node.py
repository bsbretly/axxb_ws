#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
# from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion #Quaternion in the form (w, x, y, z)

class TransformInterface():
    def __init__(self):
        self.vicon_transform = np.zeros((4, 4)) #np.arange(16).reshape(4, 4)
        self.camera_transform = np.zeros((4, 4)) #np.arange(16).reshape(4, 4)
        self.A = np.zeros((4, 4))
        self.A_prev = np.zeros((4, 4))
        self.A_list = []
        self.B = np.zeros((4, 4))
        self.B_prev = np.zeros((4, 4))
        self.B_list = []
        self.count = 1
        self.vicon_odom = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.vicon_odom_callback)
        self.camera_odom = rospy.Subscriber("/camera/odom/sample", Odometry, self.camera_odom_callback)
        self.timer = rospy.Timer(rospy.Duration(1), self.generate_data)

    def vicon_odom_callback(self, vicon_odom):
        vicon_rotation = Quaternion(vicon_odom.pose.pose.orientation.w, 
                                        vicon_odom.pose.pose.orientation.x, 
                                        vicon_odom.pose.pose.orientation.y, 
                                        vicon_odom.pose.pose.orientation.z).rotation_matrix
        vicon_translation = np.array([[vicon_odom.pose.pose.position.x,
                                       vicon_odom.pose.pose.position.y,
                                       vicon_odom.pose.pose.position.z]])
        #4x4 homogeneous transformation matrix A
        self.A = np.concatenate((np.concatenate((vicon_rotation,vicon_translation.T),axis=1), np.array([[0,0,0,1]])),axis=0)

    def camera_odom_callback(self, camera_odom):
        camera_rotation = Quaternion(camera_odom.pose.pose.orientation.w, 
                                        camera_odom.pose.pose.orientation.x, 
                                        camera_odom.pose.pose.orientation.y, 
                                        camera_odom.pose.pose.orientation.z).rotation_matrix
        camera_translation = np.array([[camera_odom.pose.pose.position.x,
                                       camera_odom.pose.pose.position.y,
                                       camera_odom.pose.pose.position.z]])
        #4x4 homogeneous transformation matrix B
        self.B = np.concatenate((np.concatenate((camera_rotation,camera_translation.T),axis=1), np.array([[0,0,0,1]])),axis=0)

    def generate_data(self, timer):
        #if bag is paused or stops playing, write data file
        if (self.A == self.A_prev).all() and np.count_nonzero(self.A):
            print("writing data file")
            with open('A.txt', 'w') as f:
                for item in self.A_list:
                    print >> f, item        
            with open('B.txt', 'w') as f:
                for item in self.B_list:
                    print >> f, item 
            print("data files written")
            rospy.signal_shutdown("data files written")            

        if self.count > 1:
            A_bar = np.dot(np.linalg.inv(self.A_prev),self.A)
            B_bar = np.dot(np.linalg.inv(self.B_prev),self.B)
            self.A_list.append(A_bar) 
            self.B_list.append(B_bar)
        else:
            A_prev = self.A
            B_prev = self.B

        if np.count_nonzero(self.A):    
            self.A_prev = self.A
            self.B_prev = self.B
            self.count += 1
        else: 
            rospy.loginfo("no input data, begin playing the bag file")

if __name__ == '__main__':
  rospy.init_node('transform_interface_node')
  try:
    TransformInterface()
    rospy.spin()
  except rospy.ROSInterruptException:
    print("exception thrown")
    pass