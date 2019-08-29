#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
# from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion #Quaternion in the form (w, x, y, z)
import pickle

class TransformInterface():
    def __init__(self):
        self.vicon_transform = np.zeros((4, 4)) #np.arange(16).reshape(4, 4)
        self.camera_transform = np.zeros((4, 4)) #np.arange(16).reshape(4, 4)
        self.T_w_b = np.zeros((4, 4))
        self.T_w_b_prev = np.zeros((4, 4))
        self.T_w_b_list = []
        self.T_i_c = np.zeros((4, 4))
        self.T_i_c_prev = np.zeros((4, 4))
        self.T_i_c_list = []
        self.vicon_quat = np.zeros((1, 4))
        self.vicon_quat_list = []
        self.camera_quat = np.zeros((1, 4))
        self.camera_quat_list = []

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
        #4x4 homogeneous transformation matrix of the robot CoM expressed in w framep
        self.T_w_b = np.concatenate((np.concatenate((vicon_rotation,vicon_translation.T),axis=1), np.array([[0,0,0,1]])),axis=0)
        self.vicon_quat = np.array([vicon_odom.pose.pose.orientation.w, 
                                        vicon_odom.pose.pose.orientation.x, 
                                        vicon_odom.pose.pose.orientation.y, 
                                        vicon_odom.pose.pose.orientation.z])

    def camera_odom_callback(self, camera_odom):
        camera_rotation = Quaternion(camera_odom.pose.pose.orientation.w, 
                                        camera_odom.pose.pose.orientation.x, 
                                        camera_odom.pose.pose.orientation.y, 
                                        camera_odom.pose.pose.orientation.z).rotation_matrix
        camera_translation = np.array([[camera_odom.pose.pose.position.x,
                                       camera_odom.pose.pose.position.y,
                                       camera_odom.pose.pose.position.z]])
        #4x4 homogeneous transformation matrix of the camera expressed in i frame
        self.T_i_c = np.concatenate((np.concatenate((camera_rotation,camera_translation.T),axis=1), np.array([[0,0,0,1]])),axis=0)
        self.camera_quat = np.array([camera_odom.pose.pose.orientation.w, 
                                        camera_odom.pose.pose.orientation.x, 
                                        camera_odom.pose.pose.orientation.y, 
                                        camera_odom.pose.pose.orientation.z])

    def generate_data(self, timer):
        #if bag is paused or stops playing, write data file
        if (self.T_w_b == self.T_w_b_prev).all() and np.count_nonzero(self.T_w_b):
            print("writing data files")
            with open('output/T_w_b', 'wb') as f:
                pickle.dump(self.T_w_b_list, f)
            with open('output/T_i_c', 'wb') as f:
                pickle.dump(self.T_i_c_list, f)
            with open('output/vicon_quat', 'wb') as f:
                pickle.dump(self.vicon_quat_list, f)
            with open('output/camera_quat', 'wb') as f:
                pickle.dump(self.camera_quat_list, f)
            print("data files written")
            rospy.signal_shutdown("data files written") 
        elif np.count_nonzero(self.T_w_b) == 0:
            rospy.loginfo("no input data, begin playing the bag file")
        else:
            self.T_w_b_list.append(self.T_w_b) 
            self.T_i_c_list.append(self.T_i_c)

            self.vicon_quat_list.append(self.vicon_quat)
            self.camera_quat_list.append(self.camera_quat)
            
            self.T_w_b_prev = self.T_w_b
            self.T_i_c_prev = self.T_i_c

if __name__ == '__main__':
  rospy.init_node('transform_interface_node')
  try:
    TransformInterface()
    rospy.spin()
  except rospy.ROSInterruptException:
    print("exception thrown")
    pass