#!/usr/bin/env python
import copy
import math
import numpy as np
import random
import pickle
import cope.SE3lib as SE3
import cope.axxbcovariance as axxb
import matplotlib.pyplot as plt
import math
from pyquaternion import Quaternion

#helper functions
def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

#ensure quaternions are sufficiently different for computation of A,B matrices
angle_threshold = 30*(math.pi/180)
# Read data files
filename = "output/T_w_b"
T_w_b =  pickle.load(open( filename, "rb" ) )

filename = "output/vicon_quat"
vicon_quat =  pickle.load(open( filename, "rb" ) )

filename = "output/T_i_c"
T_i_c =  pickle.load(open( filename, "rb" ) )

filename = "output/camera_quat"
camera_quat =  pickle.load(open( filename, "rb" ) )

A = np.dot(np.linalg.inv(T_w_b[0]),T_w_b[20])
B = np.dot(np.linalg.inv(T_i_c[0]),T_i_c[20])

# datasize = len(pattern_tfs)
datasize = len(T_w_b)
# ksamples = 30
ksamples = 50
# iters = 500
iters = 5000
Rxlist = []
sigmaRx_list = []
txlist = []
sigmatx_list = []
for n in range(iters):
    alpha = []
    beta = []
    ta = []
    tb = []
    # Generate data A and B matrices
    for i in range(ksamples):
        rand_number_1 = int(np.random.uniform(0,datasize))
        rand_number_2 = int(np.random.uniform(0,datasize))

        # print abs(angle_between(vicon_quat[rand_number_1],vicon_quat[rand_number_2]))*180/math.pi
        # print abs(angle_between(camera_quat[rand_number_1],camera_quat[rand_number_2]))*180/math.pi

        while (abs(angle_between(vicon_quat[rand_number_1],vicon_quat[rand_number_2])) < angle_threshold or 
               abs(angle_between(camera_quat[rand_number_1],camera_quat[rand_number_2])) < angle_threshold):
            # print "angle threshold failed, try another sample"
            rand_number_2 = int(np.random.uniform(0,datasize))
            # print abs(angle_between(vicon_quat[rand_number_1],vicon_quat[rand_number_2]))*180/math.pi
            # print abs(angle_between(camera_quat[rand_number_1],camera_quat[rand_number_2]))*180/math.pi

        # A = np.dot(robot_tfs[rand_number_1],np.linalg.inv(robot_tfs[rand_number_2]))
        # B = np.dot(pattern_tfs[rand_number_1],np.linalg.inv(pattern_tfs[rand_number_2]))
        A = np.dot(np.linalg.inv(T_w_b[rand_number_1]),T_w_b[rand_number_2])
        B = np.dot(np.linalg.inv(T_i_c[rand_number_1]),T_i_c[rand_number_2])
        # print "A =",A
        # print "B =",B
        alpha.append(SE3.RotToVec(A[:3,:3])) #rotation A
        beta.append(SE3.RotToVec(B[:3,:3])) #rotation B
        ta.append(A[:3,3]) #translation A
        tb.append(B[:3,3]) #translation B

    Rxinit,txinit = axxb.FCParkSolution(alpha,beta,ta,tb)

print Rxinit
print txinit
Rxinit_inv = np.zeros((3, 3))
Rxinit_inv = np.linalg.inv(Rxinit)

print Quaternion(matrix=Rxinit_inv)