#!/usr/bin/env python
import copy
import math
import numpy as np
import random
import pickle
import cope.SE3lib as SE3
import cope.axxbcovariance as axxb
import matplotlib.pyplot as plt

# Read data files
# filename = "data/pattern_tfs"
# pattern_tfs =  pickle.load(open( filename, "rb" ) )
# print("pattern_tfs type= ", type(pattern_tfs))
# print("pattern_tfs length= ", len(pattern_tfs))
# print("pattern_tfs[0] = ", pattern_tfs[0])
# print("of type ", type(pattern_tfs[0]))
# print("of size", pattern_tfs[0].shape)
filename = "../T_w_b"
T_w_b =  pickle.load(open( filename, "rb" ) )
# print(T_w_b)
# print(np.dot(np.linalg.inv(T_w_b[0]),T_w_b[1]))
# print("T_w_b type= ", type(T_w_b))
# print("T_w_b length= ", len(T_w_b))
# print("T_w_b[0] = ", T_w_b[0])
# print("of type ", type(T_w_b[0]))
# print("of size", T_w_b[0].shape)
# filename = "data/robot_tfs"
# robot_tfs =  pickle.load(open( filename, "rb" ) )
filename = "../T_i_c"
T_i_c =  pickle.load(open( filename, "rb" ) )
# print(T_i_c[0])
# print(np.dot(np.linalg.inv(T_i_c[0]),T_i_c[1]))

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
        # note this
        rand_number_1 = int(np.random.uniform(0,datasize))
        rand_number_2 = int(np.random.uniform(0,datasize))
        while rand_number_1==rand_number_2:
            rand_number_2 = int(np.random.uniform(0,datasize))

        # A = np.dot(robot_tfs[rand_number_1],np.linalg.inv(robot_tfs[rand_number_2]))
        # B = np.dot(pattern_tfs[rand_number_1],np.linalg.inv(pattern_tfs[rand_number_2]))

        A = np.dot(np.linalg.inv(T_w_b[rand_number_1]),T_w_b[rand_number_2])
        B = np.dot(np.linalg.inv(T_i_c[rand_number_1]),T_i_c[rand_number_2])
        # print("A")
        # print(A)
        # print("B")
        # print(B)
        alpha.append(SE3.RotToVec(A[:3,:3])) #rotation A
        # print(alpha)
        beta.append(SE3.RotToVec(B[:3,:3])) #rotation B
        ta.append(A[:3,3]) #translation A
        tb.append(B[:3,3]) #translation B
    Rxinit,txinit = axxb.FCParkSolution(alpha,beta,ta,tb)

print(Rxinit)
print(txinit)

