#!/usr/bin/env python

import sys
import rospy
from hybrid_control_api.srv import *
import numpy as np
import math
from scipy import linalg
from keras.models import load_model
from keras.models import Model
from utils import *

# cartesian position of the end effector
endEffectorPosition = (    ['end_arm_position_x', 0.0],   
                            ['end_arm_position_y', 0.0],   
                            ['end_arm_position_z', 0.0],   
                            ['end_arm_orientation_x', 0.0],
                            ['end_arm_orientation_y', 0.0],
                            ['end_arm_orientation_z', 0.0],
                            ['end_arm_orientation_w', 0.0]
                        )

jacobian = np.zeros(shape=(6,7))

# Euristically defined
SH_P1_LIMIT = 1.6
SH_R_LIMIT = 1.6
SH_P2_LIMIT = 1.6
EL_Y_LIMIT = 1.3
WR_R_LIMIT = 2.8
WR_Y_LIMIT = 1.4
WR_P_LIMIT = 1.4

# Create the message for the forward/jacobian package. The seven joints of the arm and the desired catesian 
# position/orientation.
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

# service to call the jacobian/forward package
def call_ik_service(currJointPose, targetPosition, targetOrientation, deltaEnd):
    rospy.init_node('hybridControl', anonymous=True)
    rospy.wait_for_service('hybrid_control_api/hybrid')
    cjp, dp = create_msg(currJointPose, targetPosition, targetOrientation)
    calc_ik_function = rospy.ServiceProxy('hybrid_control_api/hybrid', hybrid)
    res = calc_ik_function(cjp,dp)
    
    # brush the response
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
    # Makes the jacobian inverse
    jacobianTrans = np.dot(jacobian, np.matrix.transpose(jacobian))
    inverseJacobian = np.dot(np.matrix.transpose(jacobian), np.linalg.inv(jacobianTrans))

    # Convert the quartenion desired to euler angles
    xEuler, yEuler, zEuler = quaternion_to_euler_angle(deltaEnd[3], deltaEnd[4], deltaEnd[5], deltaEnd[6])
    deltaEndEuler = deltaEnd[:-1]   
    deltaEndEuler[3] = xEuler
    deltaEndEuler[4] = yEuler
    deltaEndEuler[5] = zEuler

    deltaAngles = np.dot(inverseJacobian, deltaEndEuler)
       
    return deltaAngles  

def writeToFile(name, calculated, desired, angles):
    file = open(name+".txt","a")
    string = str(calculated[0])+','+str(calculated[1])+','+str(calculated[2])+','\
            +str(calculated[3])+','+str(calculated[4])+','+str(calculated[5])+','+str(calculated[6])+','\
            +str(desired)+','\
            +str(angles[0])+','+str(angles[1])+','+str(angles[2])+','+str(angles[3])+','\
            +str(angles[4])+','+str(angles[5])+','+str(angles[6])

    file.write(str(string+'\n'))
    file.close()

def writeDistanceToFile(name, d):
    file = open(name+".txt","a")    
    file.write(str(d)+'\n')
    file.close()
 
def spiral(stepSize, radius, endEffectorPosition, numberSpirals):
    endEffector = np.zeros(shape=(int(2*math.pi*radius/stepSize)*numberSpirals,7))
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

def limitVector(desiredPosition, realEndEffector, NETORJACOBIAN_POS_THRESHOLD, MAX_STEP_JACOBIAN, MAX_STEP_NET):
    #TO DO
    delta = (desiredPosition-realEndEffector)

    if np.linalg.norm((desiredPosition-realEndEffector)[0:3]) < NETORJACOBIAN_POS_THRESHOLD:
        while True:
            if np.max(abs(delta)) > MAX_STEP_JACOBIAN:
                delta = delta - delta/100                
            else:
                break
    else:
        while True:            
            if np.max(abs(delta)) > MAX_STEP_NET:
                delta = delta - delta/100                 
            else:
                break

    return delta

if __name__ == '__main__':

    np.set_printoptions(suppress=True) 
           
    step = 0.0001
    stepSize = 0.0001
    maxStep = 0.1
    radiusArrived = stepSize*10
    maxDistance = 0.09

    maxCount = 10000
    
    endEffector = np.zeros(shape=(7,7))
    endEffector[0] = np.array([ 0.43, 0.462, 0.121, 1.0, 0.0, 0.0, 0.0 ])
    endEffector[1] = np.array([ 0.53, 0.462, 0.121, 1.0, 0.0, 0.0, 0.0 ])
    endEffector[2] = np.array([ 0.53, 0.362, 0.121, 1.0, 0.0, 0.0, 0.0 ])
    endEffector[3] = np.array([ 0.53, 0.362, 0.021, 1.0, 0.0, 0.0, 0.0 ])
    endEffector[4] = np.array([ 0.43, 0.362, 0.021, 1.0, 0.0, 0.0, 0.0 ])
    endEffector[5] = np.array([ 0.43, 0.362, 0.121, 1.0, 0.0, 0.0, 0.0 ])
    endEffector[6] = np.array([ 0.43, 0.462, 0.121, 1.0, 0.0, 0.0, 0.0 ])

    MODE = "Jacobian"

    NETORJACOBIAN_POS_THRESHOLD = 100
    MAX_STEP_NET = 0.03

    model = load_model("/home/ricardo/catkin_ws/src/hybrid_control_api/scripts/model__340_2048_Adam_sigmoid_150_120_90_75_65_55_7_10000000_0.013154525557590856_0.01312057321594821_0.0007109376021267358.h5")

    # full
    highestList = np.array([   0.0996578,  0.09700995, 0.09993384, 0.33062578, 0.28312842, 0.31182562,\
                                    0.27920513, 1.59999821, 1.59999794, 1.59999969, 1.29999984, 2.79999662,\
                                    1.3999988 , 1.3999999 , 0.08726649, 0.08726648, 0.0872665,  0.0872665,\
                                    0.08726646, 0.0872665,  0.08726644])

    lowestList = np.array([    -0.09980583, -0.09704175, -0.09984278, -0.31763874, -0.28009778, -0.31739168,\
                                    -0.28857708, -1.59999977, -1.59999941, -1.59999798, -1.29999983, -2.79999956,\
                                    -1.39999721, -1.39999988, -0.08726647, -0.08726649, -0.0872665,  -0.08726644,\
                                    -0.08726649, -0.08726646, -0.08726647])
        
    while step <= maxStep:
        File = MODE+str(step)

        angles = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 

        # The current joint pose to pass to the rospackage
        currJointPose = [  ('l_arm_sh_p1', angles[0]),\
                                ('l_arm_sh_r', angles[1]),\
                                ('l_arm_sh_p2', angles[2]),\
                                ('l_arm_el_y', angles[3]),\
                                ('l_arm_wr_r', angles[4]),\
                                ('l_arm_wr_y', angles[5]),\
                                ('l_arm_wr_p', angles[6])
                        ]

        writeDistanceToFile(File, step)
    
        i = 1  
        k = 0
        count = 0

        delta = limitVector(endEffector[i], endEffector[i-1], NETORJACOBIAN_POS_THRESHOLD, step, step) 
        idealDelta = delta  

        realEndEffector = np.array([ 0.43, 0.462, 0.121, 1.0, 0.0, 0.0, 0.0 ])
        
        while True:
            if np.linalg.norm((endEffector[i]-realEndEffector)[0:3]) < NETORJACOBIAN_POS_THRESHOLD:
                angles += call_ik_service(currJointPose, np.zeros(shape=(3)), np.zeros(shape=(4)), delta)
            else:
                predictArray = np.array([   delta[0],
                                        delta[1],
                                        delta[2],
                                        delta[3],
                                        delta[4],
                                        delta[5],
                                        delta[6],                             
                                        angles[0],
                                        angles[1],
                                        angles[2],
                                        angles[3],
                                        angles[4],
                                        angles[5],
                                        angles[6]                                
                                        ])
                # normalize the predict array
                predictArray = normalizeArray(predictArray, highestList[:14], lowestList[:14])
                # the net output
                deltaAnglesPredicted = (np.array(model.predict(predictArray.reshape(1,14)))[0])  
            
                # desnormalize the predicted angles
                deltaAnglesPredicted = desnormalizeArray(deltaAnglesPredicted, highestList[14:], lowestList[14:])
                print("Net")
                # the new angle plus the delta angles
                angles += deltaAnglesPredicted.reshape(1,7)[0] 

            # The current joint pose to pass to the rospackage
            currJointPose = [  ('l_arm_sh_p1', angles[0]),\
                                ('l_arm_sh_r', angles[1]),\
                                ('l_arm_sh_p2', angles[2]),\
                                ('l_arm_el_y', angles[3]),\
                                ('l_arm_wr_r', angles[4]),\
                                ('l_arm_wr_y', angles[5]),\
                                ('l_arm_wr_p', angles[6])
                        ]

            call_ik_service(currJointPose, np.zeros(shape=(3)), np.zeros(shape=(4)), delta)

            print ("P: "+str(endEffectorPosition[0:][0][1])+", "+str(endEffectorPosition[1:][0][1])+", "\
                            +str(endEffectorPosition[2:][0][1])+", "+str(endEffectorPosition[3:][0][1])+", "\
                            +str(endEffectorPosition[4:][0][1])+", "+str(endEffectorPosition[5:][0][1])+", "\
                            +str(endEffectorPosition[6:][0][1])   )
                
            print ("Plinha: "+str(endEffector[i]))

            realEndEffector = np.array([endEffectorPosition[0:][0][1], endEffectorPosition[1:][0][1], \
                                            endEffectorPosition[2:][0][1], endEffectorPosition[3:][0][1], \
                                            endEffectorPosition[4:][0][1], endEffectorPosition[5:][0][1], \
                                            endEffectorPosition[6:][0][1]])
            
            print("Error: "+str(np.linalg.norm((endEffector[i]-realEndEffector)[0:3])))

            #print(delta)
                        
            #raw_input()

            if np.linalg.norm((endEffector[i]-realEndEffector)[0:3]) < radiusArrived:
                print("Arrived")
                i += 1
                k = 0                                

                if i == endEffector.shape[0]:
                    #raw_input()
                    break
                else:
                    idealDelta = limitVector(endEffector[i], endEffector[i-1], NETORJACOBIAN_POS_THRESHOLD, step, step) 

            delta = limitVector(endEffector[i], realEndEffector, NETORJACOBIAN_POS_THRESHOLD, step, step) 

            if np.max(abs(idealDelta))*(k+1) >= maxDistance:
                idealDelta = endEffector[i] - endEffector[i-1] 
                k = 1
            else:
                k += 1

            writeToFile(File, endEffector[i-1]+idealDelta*k,
                                str(endEffectorPosition[0:][0][1])+","+str(endEffectorPosition[1:][0][1])+","\
                                +str(endEffectorPosition[2:][0][1])+","+str(endEffectorPosition[3:][0][1])+","\
                                +str(endEffectorPosition[4:][0][1])+","+str(endEffectorPosition[5:][0][1])+","\
                                +str(endEffectorPosition[6:][0][1]), angles)     

            count += 1

            if count == maxCount:
                break
        
        step += stepSize
               