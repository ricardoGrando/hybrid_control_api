#!/usr/bin/env python

import sys
import rospy
from thormang_ik_api.srv import *
import random
import time
import click
import numpy as np
import math

import numpy as np

from keras.optimizers import *
from keras.models import Sequential
from keras.layers import *
from keras.models import load_model
from keras.models import Model

import pickle

armKinematicsForward = (    ['sh_p1_position_x', 0.0],   
                            ['sh_p1_position_y', 0.0],   
                            ['sh_p1_position_z', 0.0],   
                            ['sh_p1_orientation_x', 0.0],
                            ['sh_p1_orientation_y', 0.0],
                            ['sh_p1_orientation_z', 0.0],
                            ['sh_p1_orientation_w', 0.0],
                            ['sh_r_position_x', 0.0],    
                            ['sh_r_position_y', 0.0],    
                            ['sh_r_position_z', 0.0],    
                            ['sh_r_orientation_x', 0.0], 
                            ['sh_r_orientation_y', 0.0], 
                            ['sh_r_orientation_z', 0.0], 
                            ['sh_r_orientation_w', 0.0],
                            ['sh_p2_position_x', 0.0],   
                            ['sh_p2_position_y', 0.0],   
                            ['sh_p2_position_z', 0.0],   
                            ['sh_p2_orientation_x', 0.0],
                            ['sh_p2_orientation_y', 0.0],
                            ['sh_p2_orientation_z', 0.0],
                            ['sh_p2_orientation_w', 0.0],
                            ['el_y_position_x', 0.0],    
                            ['el_y_position_y', 0.0],    
                            ['el_y_position_z', 0.0],    
                            ['el_y_orientation_x', 0.0], 
                            ['el_y_orientation_y', 0.0], 
                            ['el_y_orientation_z', 0.0], 
                            ['el_y_orientation_w', 0.0],
                            ['wr_r_position_x', 0.0],    
                            ['wr_r_position_y', 0.0],    
                            ['wr_r_position_z', 0.0],    
                            ['wr_r_orientation_x', 0.0], 
                            ['wr_r_orientation_y', 0.0], 
                            ['wr_r_orientation_z', 0.0], 
                            ['wr_r_orientation_w', 0.0],
                            ['wr_y_position_x', 0.0],    
                            ['wr_y_position_y', 0.0],    
                            ['wr_y_position_z', 0.0],    
                            ['wr_y_orientation_x', 0.0], 
                            ['wr_y_orientation_y', 0.0], 
                            ['wr_y_orientation_z', 0.0], 
                            ['wr_y_orientation_w', 0.0],                                                                                                                                                                           
                            ['wr_p_position_x', 0.0],    
                            ['wr_p_position_y', 0.0],     
                            ['wr_p_position_z', 0.0],    
                            ['wr_p_orientation_x', 0.0], 
                            ['wr_p_orientation_y', 0.0], 
                            ['wr_p_orientation_z', 0.0], 
                            ['wr_p_orientation_w', 0.0],  
                            ['end_arm_position_x', 0.0],   
                            ['end_arm_position_y', 0.0],   
                            ['end_arm_position_z', 0.0],   
                            ['end_arm_orientation_x', 0.0],
                            ['end_arm_orientation_y', 0.0],
                            ['end_arm_orientation_z', 0.0],
                            ['end_arm_orientation_w', 0.0]  )

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

def call_ik_service(currJointPose, whichArm, targetPosition, targetOrientation):
    rospy.init_node('demo', anonymous=True)
    #print('Waiting for the thormang_ik_api/calc_ik service')
    rospy.wait_for_service('thormang_ik_api/calc_ik')
    #print('Creating the message')
    wa, cjp, dp = create_msg(whichArm, currJointPose, targetPosition, targetOrientation)
    #print('Calling the thormang_ik_api/calc_ik service')
    calc_ik_function = rospy.ServiceProxy('thormang_ik_api/calc_ik', calc_ik)
    res = calc_ik_function(wa,cjp,dp)
    a = str(res)
    b = a.split(' ')

    arruama aqi
    j = 0
    jacobianTemp = []
    for i in range(0, len(b)):
        # if j > len(armKinematicsForward) - 1:            
        #     if b[i] == "value:":
        #         jacobianTemp.append(float(b[i+1][:-1]))
        #         #print jacobianTemp
        # else:
        if b[i] == "value:":
            #print (b[i])
            armKinematicsForward[j][1] = float(b[i+1][:-1])
            j += 1
    # # gets the jacobian matrix
    # i = 0
    # j = 0
    # while True:
    #     jacobian[j][:] = jacobianTemp[i:i+7]
        
    #     i += 7
    #     j += 1
    #     if i == 42:
    #         break        
    
    print ("Real End effector: "+str(armKinematicsForward[-7:][0][1])+", "+str(armKinematicsForward[-6:][0][1])+", "\
                            +str(armKinematicsForward[-5:][0][1])+", "+str(armKinematicsForward[-4:][0][1])+", "\
                            +str(armKinematicsForward[-3:][0][1])+", "+str(armKinematicsForward[-2:][0][1])   )
    
def writeToFile(name, calculated, desired, angles):
    file = open(name+".txt","a")
    string = str(calculated[0])+','+str(calculated[1])+','+str(calculated[2])+','\
            +str(calculated[3])+','+str(calculated[4])+','+str(calculated[5])+','\
            +str(desired)+','\
            +str(angles[0])+','+str(angles[1])+','+str(angles[2])+','+str(angles[3])+','\
            +str(angles[4])+','+str(angles[5])+','+str(angles[6])

    file.write(str(string+'\n'))
    file.close()

def readFromNetOutput(datasetPathFile):

    with open(datasetPathFile) as f:
        lines = f.readlines()
        endEffector = np.zeros(shape=(len(lines),6))
        calc_angles = np.zeros(shape=(len(lines),7))        
        for i in range(0, len(lines)):
            a = lines[i].split(',')
            
            endEffector[i] = np.array([ float(a[0]), float(a[1]), float(a[2]), float(a[3]), float(a[4]), float(a[5]) ])
            calc_angles[i] = np.array([ float(a[6]), float(a[7]), float(a[8]), float(a[9]), float(a[10]), float(a[11]), float(a[12]) ]) 
        
    return calc_angles, endEffector 

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
                
    return endEffector, d

def normalizeArray(list, highestList, lowestList):
  newList = np.zeros(shape=(13,1))
  for i in range(0, list.shape[0]):
    #print (list[i])
    #print (highestList[i])
    newList[i] = ((list[i] - lowestList[i])/(highestList[i] - lowestList[i]))*(0.9-(0.1)) + 0.1 
                
  return newList
 
def desnormalizeArray(list, highestList, lowestList):
  newList = np.zeros(shape=(7,1))
  for i in range(0, list.shape[0]):
    newList[i] = ((list[i]-0.1)*(highestList[i] - lowestList[i])/((0.9-(0.1))) + lowestList[i])
            
  return newList

if __name__ == '__main__':
    
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
    
    modelName = "model__700_8192_Adam_sigmoid_75_60_50_45_37_32_25_20_17_15_12_7_5_7_11395514_0.014489914735576766_0.013899387522263787_0.0006769247732130652.h5"
    path = "/home/ricardo/catkin_ws/net_jacobian/"
    model = load_model(path+str(modelName))

    #lowestList = pickle.load(open(path+"lowestsigmoid_5_0.15.pickle", "rb" ) )
    lowestList = np.array([ -0.14999959, -0.14999959, -0.14999959, -0.14999959, -0.14999994, -0.1499976,\
                            -1.59999977, -1.59999941, -1.59999826, -1.29999983, -2.79999475, -1.39999367,\
                            -1.39999988, -0.08726638, -0.08726638, -0.08726639, -0.08726642, -0.08726644,\
                            -0.08726644, -0.08726643])

    highestList = np.array([0.14999705, 0.14999705, 0.14999705, 0.14999705, 0.14999926, 0.14999818,\
                            1.59999821, 1.59999259, 1.59999969, 1.29999984, 2.79999521, 1.3999988,\
                            1.3999999,  0.0872664,  0.08726647, 0.08726649, 0.08726641, 0.08726649,\
                            0.08726648, 0.08726649])
                                
    print(lowestList)

    # the spiral radius and the step size
    radius = 0.1
    stepSize = radius/200
    step = radius/200

    while stepSize < radius:
        # The end effector initial position and its joint relation
        endEffectorInitialPosition = np.array([ 0.43, 0.462, 0.121, 1.0, 0.0, 0.0 ])
        angles = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Calculate the end desired endeffector based on a initial endeffector
        endEffector, d = spiral(stepSize, radius, endEffectorInitialPosition, 2) 
                
        # the output file
        netOutputFile = "txts/"+str(int(stepSize/step)-1)+".txt"
        file = open(netOutputFile,"w")

        i = 0
        while i < len(endEffector):
            if i == 0:               
                delta = (endEffector[i] - endEffectorInitialPosition)            
            else:
                call_ik_service(currJointPose, whichArm, angles[0:3], angles[3:7])
            
                realEndEffector = np.array([armKinematicsForward[-7:][0][1], armKinematicsForward[-6:][0][1], \
                                            armKinematicsForward[-5:][0][1], armKinematicsForward[-4:][0][1], \
                                            armKinematicsForward[-3:][0][1], armKinematicsForward[-2:][0][1]])
    
                delta = (endEffector[i] - realEndEffector)

            # the array to be predicted
            predictArray = np.array([     delta[0],
                                        delta[1],
                                        delta[2],
                                        delta[3],
                                        delta[4],
                                        delta[5],                             
                                        angles[0],
                                        angles[1],
                                        angles[2],
                                        angles[3],
                                        angles[4],
                                        angles[5],
                                        angles[6]                                
                                    ])
            # normalize the predict array
            predictArray = normalizeArray(predictArray, highestList[:13], lowestList[:13])
            # the net output
            deltaAnglesPredicted = (np.array(model.predict(predictArray.reshape(1,13)))[0])  
        
            # desnormalize the predicted angles
            deltaAnglesPredicted = desnormalizeArray(deltaAnglesPredicted, highestList[13:], lowestList[13:])
            # the new angle plus the delta angles
            angles += deltaAnglesPredicted.reshape(1,7)[0]
            
            # append to the output file
            file = open(netOutputFile,"a")
            string = str(endEffector[i][0])+','+str(endEffector[i][1])+','+str(endEffector[i][2])+','\
                    +str(endEffector[i][3])+','+str(endEffector[i][4])+','+str(endEffector[i][5])+','\
                    +str(angles[0])+','+str(angles[1])+','+str(angles[2])+','+str(angles[3])+','\
                    +str(angles[4])+','+str(angles[5])+','+str(angles[6])

            file.write(str(string+'\n'))
            file.close()

            i += 1  
                
        stepSize += step

    radius = 0.1
    stepSize = radius/200
    step = radius/200
    fileName = 0

    while fileName < 199:

        calculatedAngles, desEndEffector = readFromNetOutput("/home/ricardo/catkin_ws/net_jacobian/txts/"+str(fileName)+".txt")

        outputFileName = "netJacobian_"+str(stepSize)

        endEffectorInitialPosition = np.array([ 0.43, 0.462, 0.121, 1.0, 0.0, 0.0 ])

        i = 0

        file = open("txts/"+outputFileName+".txt","w")    
        file.write(str(stepSize)+'\n')
        file.close()

        while i < len(desEndEffector):
            if i == 0:
                armKinematicsForward[-7:][0][1] = endEffectorInitialPosition[0]
                armKinematicsForward[-6:][0][1] = endEffectorInitialPosition[1]
                armKinematicsForward[-5:][0][1] = endEffectorInitialPosition[2]
                armKinematicsForward[-4:][0][1] = endEffectorInitialPosition[3]    
                armKinematicsForward[-3:][0][1] = endEffectorInitialPosition[4]       
                armKinematicsForward[-2:][0][1] = endEffectorInitialPosition[5]
            
            call_ik_service(currJointPose, whichArm, calculatedAngles[i][0:3], calculatedAngles[i][3:7])
            
            writeToFile("txts/"+outputFileName, desEndEffector[i],
                                str(armKinematicsForward[-7:][0][1])+","+str(armKinematicsForward[-6:][0][1])+","\
                                +str(armKinematicsForward[-5:][0][1])+","+str(armKinematicsForward[-4:][0][1])+","\
                                +str(armKinematicsForward[-3:][0][1])+","+str(armKinematicsForward[-2:][0][1]), calculatedAngles[i])
                
            print ("Desi End effector: "+str(desEndEffector[i]))
            i += 1   
        stepSize += step
        fileName += 1         
          
        