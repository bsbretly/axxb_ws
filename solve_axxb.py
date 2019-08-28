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

#helper functions
def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

#ensure quaternions are sufficiently different for computation of A,B matrices
angle_threshold = 10*(math.pi/180)
count = 1
flag = True
# Read data files
filename = "T_w_b"
T_w_b =  pickle.load(open( filename, "rb" ) )

filename = "vicon_quat"
vicon_quat =  pickle.load(open( filename, "rb" ) )

filename = "T_i_c"
T_i_c =  pickle.load(open( filename, "rb" ) )

filename = "camera_quat"
camera_quat =  pickle.load(open( filename, "rb" ) )

# datasize = len(pattern_tfs)
datasize = len(T_w_b)
# ksamples = 30
ksamples = 30
# iters = 500
iters = 500
Rxlist = []
sigmaRx_list = []
txlist = []
sigmatx_list = []
for n in range(iters):
    alpha = []
    beta = []
    ta = []
    tb = []
    # Generate data-A and B matrices
    for i in range(ksamples):
        rand_number_1 = int(np.random.uniform(0,datasize))
        rand_number_2 = int(np.random.uniform(0,datasize))

        while (abs(angle_between(vicon_quat[rand_number_1],vicon_quat[rand_number_2])) < angle_threshold or 
               abs(angle_between(camera_quat[rand_number_1],camera_quat[rand_number_2])) < angle_threshold):
            count += 1
            rand_number_2 = int(np.random.uniform(0,datasize))
            if count > datasize:
                flag = False
                break
        else: 
            continue
        break
        # A = np.dot(robot_tfs[rand_number_1],np.linalg.inv(robot_tfs[rand_number_2]))
        # B = np.dot(pattern_tfs[rand_number_1],np.linalg.inv(pattern_tfs[rand_number_2]))
        A = np.dot(np.linalg.inv(T_w_b[rand_number_1]),T_w_b[rand_number_2])
        B = np.dot(np.linalg.inv(T_i_c[rand_number_1]),T_i_c[rand_number_2])
        alpha.append(SE3.RotToVec(A[:3,:3])) #rotation A
        beta.append(SE3.RotToVec(B[:3,:3])) #rotation B
        ta.append(A[:3,3]) #translation A
        tb.append(B[:3,3]) #translation B
    if flag == False:
        print "Unsuccessfully tried",datasize,"times to find sufficiently large angle between matrices: aborting!!"
        break
    else: 
        Rxinit,txinit = axxb.FCParkSolution(alpha,beta,ta,tb)
if (flag == True):
    print(Rxinit)
    print(txinit)

