#!/usr/bin/env python

import sys
import rospy
from hybrid_control_api.srv import *
import random
import time
import click
import numpy as np
import math
from random import shuffle

from numpy import genfromtxt

import csv

endEffectorPosition = (    ['end_arm_position_x', 0.0],   
                            ['end_arm_position_y', 0.0],   
                            ['end_arm_position_z', 0.0],   
                            ['end_arm_orientation_x', 0.0],
                            ['end_arm_orientation_y', 0.0],
                            ['end_arm_orientation_z', 0.0],
                            ['end_arm_orientation_w', 0.0]  )

# Euristically defined
SH_P1_LIMIT = 1.6
SH_R_LIMIT = 1.6
SH_P2_LIMIT = 1.6
EL_Y_LIMIT = 1.3
WR_R_LIMIT = 2.8
WR_Y_LIMIT = 1.4
WR_P_LIMIT = 1.4

def quaternion_to_euler_angle(x, y, z, w):
	ysqr = y * y
	
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))
	
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))
	
	return X, Y, Z

def create_msg(currJointPose, targetPosition, targetOrientation):
    cjp = []
    time = rospy.get_time()
    for jp in currJointPose:
        cjp.append(thormang3_manipulation_module_msgs.msg.JointPose(jp[0],jp[1],time))
    tp = geometry_msgs.msg.Point(
            x=targetPosition[0],y=targetPosition[1],z=targetPosition[2])
    to = geometry_msgs.msg.Quaternion(\
            x=targetOrientation[0],y=targetOrientation[1],\
            z=targetOrientation[2],w=targetOrientation[3])
    dp = geometry_msgs.msg.Pose(tp, to)
    
    return cjp, dp

def call_ik_service(currJointPose, targetPosition, targetOrientation, deltaEnd):
    rospy.init_node('hybrid_control', anonymous=True)
    #print('Waiting for the thormang_ik_api/calc_ik service')
    rospy.wait_for_service('hybrid_control_api/hybrid')
    #print('Creating the message')
    cjp, dp = create_msg(currJointPose, targetPosition, targetOrientation)
    #print('Calling the thormang_ik_api/calc_ik service')
    calc_ik_function = rospy.ServiceProxy('hybrid_control_api/hybrid', hybrid)
    res = calc_ik_function(cjp,dp)

    jacobian = np.zeros(shape=(6,7))

    err = np.zeros(shape=(6,1))

    j = 0
    k = 0
    jacobian = np.zeros(shape=(6,7))
    err = np.zeros(shape=(6,1))

    for i in range(0, len(res.targJointPose)):
        if i >= len(endEffectorPosition)+err.shape[0]:  
            jacobian[k][j] = res.targJointPose[i].value
            j += 1
            if j == jacobian.shape[1]:
                j = 0
                k += 1
            
            if k == jacobian.shape[0]:
                break
                
            #print jacobianTemp
        elif i < err.shape[0]: 
            err[i][0] = res.targJointPose[i].value
        else: 
            endEffectorPosition[i-err.shape[0]][1] = res.targJointPose[i].value 

    # print(jacobian)
    # print(err)
    # print(endEffectorPosition)            

    # jacobianTrans = np.dot(jacobian, np.matrix.transpose(jacobian))
    # inverseJacobian = np.dot(np.matrix.transpose(jacobian), np.linalg.inv(jacobianTrans))

    # xEuler, yEuler, zEuler = quaternion_to_euler_angle(deltaEnd[3], deltaEnd[4], deltaEnd[5], deltaEnd[6])
    # deltaEndEuler = deltaEnd[:-1]   
    # deltaEndEuler[3] = xEuler
    # deltaEndEuler[4] = yEuler
    # deltaEndEuler[5] = zEuler

    # deltaAngles = np.dot(inverseJacobian, deltaEndEuler)

    # return deltaAngles 

def writeToFile(deltaCartesian, angles, deltaAngles, writer):    
    aux = np.concatenate((deltaCartesian, angles), axis=0)
    data = np.concatenate((aux, deltaAngles), axis=0)

    writer.writerow(data.tolist())

    # file = open(name+".txt","a")
    # string = str(deltaCartesian[0])+','+str(deltaCartesian[1])+','+str(deltaCartesian[2])+','+str(deltaCartesian[3])+','+str(deltaCartesian[4])+','+str(deltaCartesian[5])+','+str(deltaCartesian[6])+','+str(angles[0])+','+str(angles[1])+','+str(angles[2])+','+str(angles[3])+','+str(angles[4])+','+str(angles[5])+','+str(angles[6])+','+str(deltaAngles[0])+','+str(deltaAngles[1])+','+str(deltaAngles[2])+','+str(deltaAngles[3])+','+str(deltaAngles[4])+','+str(deltaAngles[5])+','+str(deltaAngles[6])

    # file.write(str(string+'\n'))
    # file.close() 

def writeEndEffectorToFile(endEffector, writer):
    writer.writerow(endEffector.tolist())
    # file = open(name+".txt","a")
    # string = str(endEffector[0])+','+str(endEffector[1])+','+str(endEffector[2])+','+\
    #             str(endEffector[3])+','+str(endEffector[4])+','+str(endEffector[5])+','+str(endEffector[6])

    # file.write(str(string+'\n'))
    # file.close() 

def readPreDatasetOutput(datasetPathFile):

    with open(datasetPathFile) as f:
        lines = f.readlines()
        angles = np.zeros(shape=(len(lines),7))
        index = [i for i in range(len(lines))]
        shuffle(index)
        for i in range(0, len(lines)):
            a = lines[len(lines) - 1 - i].split(',')
            angles[index[i]] = np.array([ float(a[0]), float(a[1]), float(a[2]), float(a[3]), float(a[4]), float(a[5]), float(a[6][:-1]) ])
        
    return angles

def createTypedAngles(size):

    anglesMax = np.genfromtxt("/home/ricardo/catkin_ws/src/hybrid_control_api/scripts/angles.csv", delimiter=',')

    print(anglesMax.shape)
    
    angles = np.array([     np.random.uniform(np.min(anglesMax[:,0]-0.1), np.max(anglesMax[:,0]+0.1), size),
                            np.random.uniform(np.min(anglesMax[:,1]-0.1), np.max(anglesMax[:,1]+0.1), size),
                            np.random.uniform(np.min(anglesMax[:,2]-0.1), np.max(anglesMax[:,2]+0.1), size),
                            np.random.uniform(np.min(anglesMax[:,3]-0.1), np.max(anglesMax[:,3]+0.1), size),
                            np.random.uniform(np.min(anglesMax[:,4]-0.1), np.max(anglesMax[:,4]+0.1), size),
                            np.random.uniform(np.min(anglesMax[:,5]-0.1), np.max(anglesMax[:,5]+0.1), size),
                            np.random.uniform(np.min(anglesMax[:,6]-0.1), np.max(anglesMax[:,6]+0.1), size)
                        ])
    
    print angles.shape
    return angles.T

def createValidNormalAnglesDelta(deltaAnglesMax):

    while(1):
        delta = np.array([    np.random.normal(0, deltaAnglesMax),
                                        np.random.normal(0, deltaAnglesMax),
                                        np.random.normal(0, deltaAnglesMax),
                                        np.random.normal(0, deltaAnglesMax),
                                        np.random.normal(0, deltaAnglesMax),
                                        np.random.normal(0, deltaAnglesMax),
                                        np.random.normal(0, deltaAnglesMax)
                                    ])
        
        if (np.max(abs(delta)) < deltaAnglesMax):
            return delta        
    
if __name__ == '__main__':
    #np.set_printoptions(formatter={'float':lambda x: '%+01.5f ' % x})

    # used the inverted cinematic interface. Used for anything. 
    delta_degrees_limit = 5
    deltaAnglesMax =  0.0174533*delta_degrees_limit

    deltaEndEffectorMax = 0.10

    #mudar nome    
    File = "dataset_normal"+str(delta_degrees_limit)+"_"+str(deltaEndEffectorMax)+"_typed_more_B"

    #datacsvfile = open(File+".csv", 'wb')
    datacsvfile = open(File+".csv", 'a')    
    datacsvwriter = csv.writer(datacsvfile)

    #endcsvfile = open(File+str("endEffector")+".csv", 'wb')
    endcsvfile = open(File+str("endEffector")+".csv", 'a')    
    endcsvwriter = csv.writer(endcsvfile)
    
    i = 0
    count = 0
    while True:

        #call_ik_service(currJointPose, whichArm, [0, 0, 0], [0, 0, 0, 0]) 
        
        #angles = readPreDatasetOutput("/home/ricardo/Desktop/True.txt") 
        angles = createTypedAngles(1000000)

        while i != angles.shape[0]:
            # print("---------------------------------------")    
            # print("Initial Angles:")
            # print(angles[i])          
            currJointPose = [  ('l_arm_sh_p1', angles[i][0]),\
                                ('l_arm_sh_r', angles[i][1]),\
                                ('l_arm_sh_p2', angles[i][2]),\
                                ('l_arm_el_y', angles[i][3]),\
                                ('l_arm_wr_r', angles[i][4]),\
                                ('l_arm_wr_y', angles[i][5]),\
                                ('l_arm_wr_p', angles[i][6])
                            ]

            call_ik_service(currJointPose, np.zeros(shape=(3,)), np.zeros(shape=(4,)), np.zeros(shape=(7,1)))
            
            calculatedPosition = np.array([     endEffectorPosition[0:][0][1],
                                                endEffectorPosition[1:][0][1],
                                                endEffectorPosition[2:][0][1],
                                                endEffectorPosition[3:][0][1],
                                                endEffectorPosition[4:][0][1],
                                                endEffectorPosition[5:][0][1], 
                                                endEffectorPosition[6:][0][1]
                                            ]) 
            
            # print("Callculated end effector initial position: ")
            # print(calculatedPosition)

            deltaAngles = createValidNormalAnglesDelta(deltaAnglesMax)
            # print("Aleatory delta:")
            # print(deltaAngles)  

            angles[i] += deltaAngles
            # print("Initial Angles plus aleatory delta:")
            # print(angles[i])     

            currJointPose = [  ('l_arm_sh_p1', angles[i][0]),\
                                ('l_arm_sh_r', angles[i][1]),\
                                ('l_arm_sh_p2', angles[i][2]),\
                                ('l_arm_el_y', angles[i][3]),\
                                ('l_arm_wr_r', angles[i][4]),\
                                ('l_arm_wr_y', angles[i][5]),\
                                ('l_arm_wr_p', angles[i][6])
                            ]

            call_ik_service(currJointPose, np.zeros(shape=(3,1)), np.zeros(shape=(4,1)), np.zeros(shape=(7,1)))
            
            calculatedPositionAfter = np.array([     endEffectorPosition[0:][0][1],
                                                    endEffectorPosition[1:][0][1],
                                                    endEffectorPosition[2:][0][1],
                                                    endEffectorPosition[3:][0][1],
                                                    endEffectorPosition[4:][0][1],
                                                    endEffectorPosition[5:][0][1],
                                                    endEffectorPosition[6:][0][1]
                                            ]) 
            
            # print("Callculated end effector initial position plus aleatory delta: ")
            # print(calculatedPositionAfter)

            deltaEndEffector = calculatedPositionAfter-calculatedPosition
            # print("Delta endeffector: ")
            # print(deltaEndEffector)

            # print("Initial Angles:")
            # print(angles[i]-deltaAngles)
            
            if (np.max(abs(deltaEndEffector[0:3])) < deltaEndEffectorMax): 
                if( abs(deltaAngles[6]+angles[i][6]) < 1.4 ): 
                    if (abs(deltaAngles[5]+angles[i][5]) < 1.4): 
                        if (abs(deltaAngles[4]+angles[i][4]) < 2.8):
                            if (abs(deltaAngles[3]+angles[i][3]) < 1.3): 
                                if( abs(deltaAngles[2]+angles[i][2]) < 1.6):
                                    if( abs(deltaAngles[1]+angles[i][1]) < 1.6):
                                        if(abs(deltaAngles[0]+angles[i][0]) < 1.6): 
                                            
                                            writeToFile(deltaEndEffector, angles[i]-deltaAngles, deltaAngles, datacsvwriter)
                                            # print("Writed down")
                                            #print(deltaEndEffector)
                                            # print(angles[i]-deltaAngles)
                                            # print(deltaAngles)                                            

                                            writeEndEffectorToFile(calculatedPosition, endcsvwriter)
                                            count += 1 

                                            #print(count)

            i += 1

            if i % 1000 == 0:
                print("Valid: "+str(count)+" of "+str(i)+" "+str(float(float(count)/float(i)))+"%")
            
        i = 0   
    