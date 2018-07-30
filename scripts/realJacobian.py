#!/usr/bin/env python

import sys
import rospy
from thormang_ik_api.srv import *
import random
import time
import click
import numpy as np
import math
from scipy import linalg

armKinematicsForward = (    ['end_arm_position_x', 0.0],   
                            ['end_arm_position_y', 0.0],   
                            ['end_arm_position_z', 0.0],   
                            ['end_arm_orientation_x', 0.0],
                            ['end_arm_orientation_y', 0.0],
                            ['end_arm_orientation_z', 0.0]  )

jacobian = np.zeros(shape=(6,7))

# Euristically defined
SH_P1_LIMIT = 1.6
SH_R_LIMIT = 1.6
SH_P2_LIMIT = 1.6
EL_Y_LIMIT = 1.3
WR_R_LIMIT = 2.8
WR_Y_LIMIT = 1.4
WR_P_LIMIT = 1.4

dataset_size = 1000000

def create_msg(whichArm, currJointPose, targetPosition, targetOrientation):
    wa = std_msgs.msg.String(whichArm)
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
    
    return wa, cjp, dp

def call_ik_service(currJointPose, whichArm, targetPosition, targetOrientation, deltaCartesianEnd):
    rospy.init_node('demo', anonymous=True)
    #print('Waiting for the thormang_ik_api/calc_ik service')
    rospy.wait_for_service('thormang_ik_api/calc_ik')
    #print('Creating the message')
    wa, cjp, dp = create_msg(whichArm, currJointPose, targetPosition, targetOrientation)
    #print('Calling the thormang_ik_api/calc_ik service')
    calc_ik_function = rospy.ServiceProxy('thormang_ik_api/calc_ik', calc_ik)
    res = calc_ik_function(wa,cjp,dp)
    

    arruma aqi
    j = 0
    k = 0
    for i in range(0, len(res.targJointPose)):

        if i >= len(armKinematicsForward):  
            jacobian[k][j] = res.targJointPose[i].value
            j += 1
            if j == jacobian.shape[1]:
                j = 0
                k += 1
            
            if k == jacobian.shape[0]:
                break
                
            #print jacobianTemp
        else:  
            armKinematicsForward[i][1] = res.targJointPose[i].value            

    #pseudoJacobian =  np.linalg.pinv(jacobian)  
    aux = np.dot(jacobian, np.matrix.transpose(jacobian))
    pseudoJacobian = np.dot(np.matrix.transpose(jacobian), np.linalg.inv(aux))
            
    deltaAngles = np.dot(pseudoJacobian, deltaCartesianEnd)
        
    return deltaAngles

def writeToFile(name, calculated, desired, angles):
    file = open(name+".txt","a")
    string = str(calculated[0])+','+str(calculated[1])+','+str(calculated[2])+','\
            +str(calculated[3])+','+str(calculated[4])+','+str(calculated[5])+','\
            +str(desired)+','\
            +str(angles[0])+','+str(angles[1])+','+str(angles[2])+','+str(angles[3])+','\
            +str(angles[4])+','+str(angles[5])+','+str(angles[6])

    file.write(str(string+'\n'))
    file.close()

def writeDistanceToFile(name, d):
    file = open(name+".txt","a")    
    file.write(str(d)+'\n')
    file.close()
 
def losango(steps, sideSize, endEffectorPosition):

    endEffector = np.zeros(shape=(steps,6))
    endEffector[:] = endEffectorPosition
    
    j = 0
    
    for i in range(0, endEffector.shape[0]):
        if i < endEffector.shape[0]/4:
            endEffector[i][2] += float(j*sideSize)/(endEffector.shape[0]/4)
            endEffector[i][1] += float(j*sideSize)/(endEffector.shape[0]/4)
        elif i >= endEffector.shape[0]/4 and i < endEffector.shape[0]/2:
            endEffector[i][2] += float(j*sideSize)/(endEffector.shape[0]/4) + sideSize
            endEffector[i][1] += float(-j*sideSize)/(endEffector.shape[0]/4) + sideSize
        elif i >= endEffector.shape[0]/2 and i < endEffector.shape[0]*(3.0/4.0):
            endEffector[i][2] += float(-j*sideSize)/(endEffector.shape[0]/4) + 2*sideSize
            endEffector[i][1] += float(-j*sideSize)/(endEffector.shape[0]/4)
        else:
            endEffector[i][2] += float(-j*sideSize)/(endEffector.shape[0]/4)
            endEffector[i][1] += float(j*sideSize)/(endEffector.shape[0]/4) - sideSize
        
        endEffector[i][0] -= (i*sideSize)/(endEffector.shape[0])
        
        j += 1
        if j == endEffector.shape[0]/4:
            j = 0    

    d = ((endEffector[1][0] - endEffector[0][0])**2 + \
        (endEffector[1][1] - endEffector[0][1])**2 + \
        (endEffector[1][2] - endEffector[0][2])**2)**(0.5)

    return endEffector

def spiralFixed(steps, radius, endEffectorPosition, numberSpirals):
    endEffector = np.zeros(shape=(int(360/steps)*numberSpirals,6))
    endEffector[:] = endEffectorPosition
    
    passo = (2*math.pi*radius)/(360/steps)
    angle = math.asin(passo/(radius))
    
    for i in range(0, endEffector.shape[0]):
        endEffector[i][2] = endEffector[i][2] - radius*passo*(i+1)       
        endEffector[i][0] = endEffector[i][0] + radius*math.cos(angle*(i+1)) - radius
        endEffector[i][1] = endEffector[i][1] + radius*math.sin(angle*(i+1))
        
    d = ((endEffector[1][0] - endEffector[0][0])**2 + \
            (endEffector[1][1] - endEffector[0][1])**2 + \
            (endEffector[1][2] - endEffector[0][2])**2)**(0.5)

    print d
    
    return endEffector, d

def spiral(stepSize, radius, endEffectorPosition, numberSpirals):
    endEffector = np.zeros(shape=(int(2*math.pi*radius/stepSize)*numberSpirals,6))
    endEffector[:] = endEffectorPosition
        
    angle = math.asin(stepSize/(radius))
    
    for i in range(0, endEffector.shape[0]):
        endEffector[i][2] = endEffector[i][2] - radius*stepSize*(i+1)       
        endEffector[i][0] = endEffector[i][0] + radius*math.cos(angle*(i+1)) - radius
        endEffector[i][1] = endEffector[i][1] + radius*math.sin(angle*(i+1))
        
        if i > 0:
            dx = (endEffector[i][0] - endEffector[i-1][0])
            dy = (endEffector[i][1] - endEffector[i-1][1])
            dz = (endEffector[i][2] - endEffector[i-1][2])
            d =  ( dx**2 + dy**2 + dz**2 )**(0.5)
            #print endEffector.shape[0], stepSize, dx, dy, dz, d
    
    return endEffector, d

def returnToTrajectory(endEffector, realEndEffector):
    d = 10000
    delta = realEndEffector
    index = 0
    for i in range(0, endEffector.shape[0]):
        auxD1 = (endEffector[i] - realEndEffector)**2
        auxD2 = (np.sum(auxD1))**0.5
        if auxD2 < d:
            d = auxD2
            index = i
            delta = endEffector[i] - realEndEffector

    print("Distance to closest: "+str(d))
    return index

if __name__ == '__main__':

    np.set_printoptions(suppress=True) 
    
    # used the inverted cinematic interface. Used for anything. 
    whichArm = 'left'
    currJointPose = [ ('l_arm_sh_p1', 2),\
                    ('l_arm_sh_r', 4),\
                    ('l_arm_sh_p2', 6),  \
                    ('l_arm_el_y', 8), \
                    ('l_arm_wr_r', 10),  \
                    ('l_arm_wr_y', 12),\
                    ('l_arm_wr_p', 14),\
                    ('end_effector', 34)]
    
    radius = 0.10
    
    stepSize = radius/20000
    print (stepSize)
        
    while stepSize <= radius:
        File = "calcJacobian_"+str(stepSize)

        angles = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])        

        endEffectorInitialPosition = np.array([ 0.43, 0.462, 0.121, 1.0, 0.0, 0.0 ])

        endEffector, d = spiral(stepSize, radius, endEffectorInitialPosition, 2)  

        writeDistanceToFile(File, d)
    
        i = 0

        delta =(endEffector[0] - endEffectorInitialPosition)
        #delta = np.array([ 0.001, 0.0, 0.0, 0.0, 0.0, 0.0])   

        while i < len(endEffector):
            
            angles += call_ik_service(currJointPose, whichArm, angles[0:3], angles[3:7], delta)

            print ("P: "+str(armKinematicsForward[0:][0][1])+", "+str(armKinematicsForward[1:][0][1])+", "\
                            +str(armKinematicsForward[2:][0][1])+", "+str(armKinematicsForward[3:][0][1])+", "\
                            +str(armKinematicsForward[4:][0][1])+", "+str(armKinematicsForward[5:][0][1])   )
                
            print ("Plinha: "+str(endEffector[i]))

            print("Deltap")
            print(delta)
            
            #call_ik_service(currJointPose, whichArm, angles[0:3], angles[3:7], delta)

            realEndEffector = np.array([armKinematicsForward[0:][0][1], armKinematicsForward[1:][0][1], \
                                            armKinematicsForward[2:][0][1], armKinematicsForward[3:][0][1], \
                                            armKinematicsForward[4:][0][1], armKinematicsForward[5:][0][1]])

            writeToFile(File, endEffector[i],
                                str(armKinematicsForward[0:][0][1])+","+str(armKinematicsForward[1:][0][1])\
                                +str(armKinematicsForward[2:][0][1])+","+str(armKinematicsForward[3:][0][1])+","\
                                +str(armKinematicsForward[4:][0][1])+","+str(armKinematicsForward[5:][0][1]), angles)
                
            
            i += 1

            print("Error: "+str(endEffector[i-1] - realEndEffector))

            #delta = np.array([ 0.001, 0.0, 0.0, 0.0, 0.0, 0.0])  
            #delta = endEffector[i] - realEndEffector  

            # if np.max(abs(delta)) > stepSize:
            #     print("Delta bigger")
            #     index = returnToTrajectory(endEffector, realEndEffector)

            #     print("Closest to: "+str(index))
            #     delta = (endEffector[index] - realEndEffector)

            #     # if np.max(abs(delta)) < stepSize/radius:
            #     #     print("Route fixed")
            #     #     delta = (endEffector[index+1] - realEndEffector)

            #     #print ("Desi End effector: "+str(endEffector[index]))
            # else:
            #     pass
            #     #print ("Desi End effector: "+str(endEffector[i]))
            
            raw_input()

            print (delta)
            print(i)
            print ("---------------")

            # if i == 100:
            #    break

            #break
        
        stepSize += 0.0005
        
        break
        