import rospy
from hybrid_control_api.srv import *
import numpy as np
from std_msgs.msg import String
import threading
from topicCartesianState import *
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from keras.models import load_model
from keras.models import Model
import csv
from runOnThormang import *
from utils import *
from tf.transformations import *

# cartesian position of the end effector
endEffectorPosition = (    ['end_arm_position_x', 0.0],   
                            ['end_arm_position_y', 0.0],   
                            ['end_arm_position_z', 0.0],   
                            ['end_arm_orientation_x', 0.0],
                            ['end_arm_orientation_y', 0.0],
                            ['end_arm_orientation_z', 0.0],
                            ['end_arm_orientation_w', 0.0]
                        )

# topics of each joint, including both grippers
pubList =  [    '/thormang3/l_arm_sh_p1_position/command', # -1.6 to 1.6
                '/thormang3/l_arm_sh_r_position/command', # -1.6 to 1.6
                '/thormang3/l_arm_sh_p2_position/command', # -1.6 to 1.6
                '/thormang3/l_arm_el_y_position/command',  # -1.3 to 1.3
                '/thormang3/l_arm_wr_r_position/command', # -2.8 to 2.8
                '/thormang3/l_arm_wr_y_position/command', # 1.4 to 1.4
                '/thormang3/l_arm_wr_p_position/command', # -1.4 to 1.4
                '/thormang3/l_arm_grip_position/command', # -1.4 to 1.4
                '/thormang3/l_arm_grip_1_position/command' # -1.4 to 1.4
                
            ]

# Number of states to change a piece between two towers
TOTAL_STATES = 9

# Cartesian position of x. It is fixed. Only y and z changes
x = 0.535
# Minimum position distance between the actual and the goal to change the state
ARRIVED_POS_THRESHOLD = 0.0005
# Minimum orientation distance for W of a orientation x, y, z towards the goal. 
ARRIVED_ORI_THRESHOLD = 0.0005
# Threshold to go with net or jacobian. If cartesian position distance towards the goal is small then it goes with jacobian 
NETORJACOBIAN_THRESHOLD = 0.015
# Max step that the net goes towards the goal. THE MAX TRAINED STEP WAS 0.1
MAX_STEP_NET = 0.03
# Max step that jacobian does towards the goal. THE SMALLER THE SLOWER BUT WITH BEST ACCURACY
MAX_STEP_JACOBIAN = 0.01
# Height in Z for each of the four pieces in the tower. The last value in the upper tower value
heightTowerList = [0.080, 0.115, 0.145, 0.175, 0.23]
# Lenght in Y for the three towers.
lengthTowerList = [0.15, 0.3, 0.45]
# Number of time to publish the same thing in order to wait the gripper to close  
GRIPPER_WAIT_MAX_TIME = 10
# Gripper closing value
GRIPPER_CLOSING = [1.0, 0.98, 0.96, 0.95]
# Number of times to publish the desired value. THE JOINTS HAVE A DELAY TO GET TO THE DESIRED POSE.
PUB_TIMES = 200

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


class hybridControl(object):
    def __init__(self):
        np.set_printoptions(suppress=True) 
    
        # The initial position of the angles and its end effector 
        self.endEffectorInitialPosition = np.array([ 0.535, 0.3, 0.121, 1.0, 0.0, 0.0, 0.0 ])
        self.angles = np.array([-1.270851663074207,0.7706552679983241,1.1857071812176423,0.690658380033005,1.076204383634198,-1.2691182991333085,-1.0401750075045482])
        
        # The current joint pose to pass to the rospackage
        self.currJointPose = [  ('l_arm_sh_p1', self.angles[0]),\
                                ('l_arm_sh_r', self.angles[1]),\
                                ('l_arm_sh_p2', self.angles[2]),\
                                ('l_arm_el_y', self.angles[3]),\
                                ('l_arm_wr_r', self.angles[4]),\
                                ('l_arm_wr_y', self.angles[5]),\
                                ('l_arm_wr_p', self.angles[6])
                            ]
        # Threads and the mutex to control them with the main
        self.linkThreads = []
        self.mutex = threading.Condition()        
        
        # Call ros service
        call_ik_service(self.currJointPose, self.endEffectorInitialPosition[0:3], self.endEffectorInitialPosition[3:7], np.zeros(shape=(7,)))

        # Creates the thread for each 
        for i in range(0, len(pubList)):
            self.linkThreads.append(topicCartesianState(pubList[i], self.mutex, False))
                
        for i in range(0, len(pubList)):
	    self.linkThreads[i].daemon = True
            self.linkThreads[i].start()

        self.thormangPublishers = runOnThormang()

        self.thormangPublishers.runOnThormang(self.angles, self.mutex, self.linkThreads, 0, PUB_TIMES)

        #arruma essa porra
        self.model = load_model("/home/ricardo/catkin_ws/src/hybrid_control_api/scripts/model__340_2048_Adam_sigmoid_150_120_90_75_65_55_7_10000000_0.013154525557590856_0.01312057321594821_0.0007109376021267358.h5")

        # full
        self.highestList = np.array([   0.0996578,  0.09700995, 0.09993384, 0.33062578, 0.28312842, 0.31182562,\
                                        0.27920513, 1.59999821, 1.59999794, 1.59999969, 1.29999984, 2.79999662,\
                                        1.3999988 , 1.3999999 , 0.08726649, 0.08726648, 0.0872665,  0.0872665,\
                                        0.08726646, 0.0872665,  0.08726644])

        self.lowestList = np.array([    -0.09980583, -0.09704175, -0.09984278, -0.31763874, -0.28009778, -0.31739168,\
                                        -0.28857708, -1.59999977, -1.59999941, -1.59999798, -1.29999983, -2.79999956,\
                                        -1.39999721, -1.39999988, -0.08726647, -0.08726649, -0.0872665,  -0.08726644,\
                                        -0.08726649, -0.08726646, -0.08726647])

        self.realEndEffector = self.endEffectorInitialPosition

        self.desiredPosition = np.array([ 0.535, 0.3, 0.16, 1.0, 0.0, 0.0, 0.0 ])

        self.delta = self.desiredPosition-self.endEffectorInitialPosition

        self.gripper = 0

        self.towers = [[4, 3, 2, 1], [], []]

        self.movementState = 0

        self.concluded = False

        csvfile = open("angles.csv", 'wb')
        self.csvwriter = csv.writer(csvfile)

        # setar no init
        self.gameState = 0

        self.publisherCounter = 0

        for i in range(0, 10):
            self.thormangPublishers.runOnThormang(self.angles, self.mutex, self.linkThreads, self.gripper, PUB_TIMES)  

    def limitVector(self):
        #TO DO
        self.delta = (self.desiredPosition-self.realEndEffector)

        if np.max(abs(self.delta)) < NETORJACOBIAN_THRESHOLD:
            while True:
                if np.max(abs(self.delta)) > MAX_STEP_JACOBIAN:
                    self.delta = self.delta/2                
                else:
                    break
        else:
            while True:            
                if np.max(abs(self.delta)) > MAX_STEP_NET:
                    self.delta = self.delta/2                
                else:
                    break

    def moveArm(self):

        # set the new delta
        self.limitVector()

        # verify the distance towards the desired
        if np.linalg.norm((self.desiredPosition-self.realEndEffector)[0:3]) > ARRIVED_POS_THRESHOLD or ((self.desiredPosition-self.realEndEffector)[-1]) > ARRIVED_ORI_THRESHOLD: # or (self.movementState == 2)  or (self.movementState == 5):
            #"Usa a rede"
            if np.linalg.norm((self.desiredPosition-self.realEndEffector)[0:3]) > NETORJACOBIAN_THRESHOLD:
                predictArray = np.array([   self.delta[0],
                                        self.delta[1],
                                        self.delta[2],
                                        self.delta[3],
                                        self.delta[4],
                                        self.delta[5],
                                        self.delta[6],                             
                                        self.angles[0],
                                        self.angles[1],
                                        self.angles[2],
                                        self.angles[3],
                                        self.angles[4],
                                        self.angles[5],
                                        self.angles[6]                                
                                        ])
                # normalize the predict array
                predictArray = normalizeArray(predictArray, self.highestList[:14], self.lowestList[:14])
                # the net output
                deltaAnglesPredicted = (np.array(self.model.predict(predictArray.reshape(1,14)))[0])  
            
                # desnormalize the predicted angles
                deltaAnglesPredicted = desnormalizeArray(deltaAnglesPredicted, self.highestList[14:], self.lowestList[14:])
                print("Net")
                # the new angle plus the delta angles
                self.angles += deltaAnglesPredicted.reshape(1,7)[0] 

                self.thormangPublishers.runOnThormang(self.angles, self.mutex, self.linkThreads, self.gripper, PUB_TIMES)           

                self.currJointPose = [  ('l_arm_sh_p1', self.angles[0]),\
                                ('l_arm_sh_r', self.angles[1]),\
                                ('l_arm_sh_p2', self.angles[2]),\
                                ('l_arm_el_y', self.angles[3]),\
                                ('l_arm_wr_r', self.angles[4]),\
                                ('l_arm_wr_y', self.angles[5]),\
                                ('l_arm_wr_p', self.angles[6])
                            ]

                call_ik_service(self.currJointPose, self.desiredPosition[0:3], self.desiredPosition[3:7], np.zeros(shape=(7,)))

                self.realEndEffector = np.array([endEffectorPosition[0:][0][1], endEffectorPosition[1:][0][1], \
                                                    endEffectorPosition[2:][0][1], endEffectorPosition[3:][0][1], \
                                                    endEffectorPosition[4:][0][1], endEffectorPosition[5:][0][1], \
                                                    endEffectorPosition[6:][0][1]])

                print("Norm total: "+str(np.linalg.norm((self.desiredPosition-self.realEndEffector))))
                print("Norm pos: "+str(np.linalg.norm((self.desiredPosition-self.realEndEffector)[0:3])))
                print("Norm ori: "+str(np.linalg.norm((self.desiredPosition-self.realEndEffector)[3:])))

                #print(self.realEndEffector)

            # Usa o Jacobiano
            else:
                self.currJointPose = [  ('l_arm_sh_p1', self.angles[0]),\
                                ('l_arm_sh_r', self.angles[1]),\
                                ('l_arm_sh_p2', self.angles[2]),\
                                ('l_arm_el_y', self.angles[3]),\
                                ('l_arm_wr_r', self.angles[4]),\
                                ('l_arm_wr_y', self.angles[5]),\
                                ('l_arm_wr_p', self.angles[6])
                            ]
                self.angles += call_ik_service(self.currJointPose, self.desiredPosition[0:3], self.desiredPosition[3:7], self.delta)

                # Send to the topic when it gets close
                #if np.linalg.norm((self.desiredPosition-self.realEndEffector)[0:3])/ARRIVED_POS_THRESHOLD < 1.5 and ((self.desiredPosition-self.realEndEffector)[-1])/ARRIVED_ORI_THRESHOLD < 1.5:
                self.thormangPublishers.runOnThormang(self.angles, self.mutex, self.linkThreads, self.gripper, PUB_TIMES)
                self.publisherCounter = 0
                print("Jacobian")
                print("Norm total: "+str(np.linalg.norm((self.desiredPosition-self.realEndEffector))))
                print("Norm pos: "+str(np.linalg.norm((self.desiredPosition-self.realEndEffector)[0:3])))
                print("Norm ori: "+str(np.linalg.norm((self.desiredPosition-self.realEndEffector)[3:])))

                    #print(self.realEndEffector)
                # else:
                #     self.publisherCounter += 1           
                
                #time.sleep(1)

                self.csvwriter.writerow(self.angles.tolist())

                self.currJointPose = [  ('l_arm_sh_p1', self.angles[0]),\
                                ('l_arm_sh_r', self.angles[1]),\
                                ('l_arm_sh_p2', self.angles[2]),\
                                ('l_arm_el_y', self.angles[3]),\
                                ('l_arm_wr_r', self.angles[4]),\
                                ('l_arm_wr_y', self.angles[5]),\
                                ('l_arm_wr_p', self.angles[6])
                            ]
                
                call_ik_service(self.currJointPose, self.desiredPosition[0:3], self.desiredPosition[3:7], self.delta)

                self.realEndEffector = np.array([endEffectorPosition[0:][0][1], endEffectorPosition[1:][0][1], \
                                                    endEffectorPosition[2:][0][1], endEffectorPosition[3:][0][1], \
                                                    endEffectorPosition[4:][0][1], endEffectorPosition[5:][0][1], \
                                                    endEffectorPosition[6:][0][1]])


                
        else:
            if (self.movementState != 2 and self.movementState != 6 and self.movementState != 7):  
                if self.movementState == TOTAL_STATES-1:
                    self.movementState = 0
                    self.concluded = True                
                else:     
                    self.movementState += 1
                
            else:
                self.thormangPublishers.runOnThormang(self.angles, self.mutex, self.linkThreads, self.gripper, PUB_TIMES)           


        #print("Move State:" + str(self.movementState))

    def setTarget(self, target):

        self.desiredPosition = target
        self.delta = (self.desiredPosition-self.realEndEffector)

    def moveBlock(self, fromTower, toTower):
        
        if self.movementState == 0:
            # go to above of the disered tower to take the block
            y = lengthTowerList[fromTower]        
            z = heightTowerList[4]

            self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ]))

        elif self.movementState == 1:
            # go down to grab the block
            y = lengthTowerList[fromTower]  

            z = heightTowerList[len(self.towers[fromTower])-1]
            self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ]))

            self.waitForGripper = 0            

        elif self.movementState == 2:
            # GRAB and wait
            
            self.gripper = GRIPPER_CLOSING[self.towers[fromTower][-1]-1]

            if self.waitForGripper <= GRIPPER_WAIT_MAX_TIME:
                self.waitForGripper += 1
            else:
                #self.waitForGripper = 0
                self.movementState += 1

                y = lengthTowerList[fromTower]        
                z = heightTowerList[4]            

                self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ])) 
        elif self.movementState == 3:
            # go up
            y = lengthTowerList[fromTower]        
            z = heightTowerList[4]            

            self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ])) 


        elif self.movementState == 4:
            # go to above of the tower we want to place the block
            y = lengthTowerList[toTower]        
            z = heightTowerList[4]

            self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ]))

        elif self.movementState == 5:
            # go down
            y = lengthTowerList[toTower]  

            z = heightTowerList[len(self.towers[toTower])]
            self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ]))

            self.waitForGripper = 0

        elif self.movementState == 6:
            # release the block and wait            

            if self.waitForGripper <= GRIPPER_WAIT_MAX_TIME:
                self.waitForGripper += 1
            else:
                #self.waitForGripper = 0
                self.movementState += 1

                self.gripper = 0

                self.waitForGripper = 0

        elif self.movementState == 7:
            if self.waitForGripper <= GRIPPER_WAIT_MAX_TIME:
                self.waitForGripper += 1
            else:
                #self.waitForGripper = 0
                self.movementState += 1

                y = lengthTowerList[toTower]        
                z = heightTowerList[4]

                self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ]))

        elif self.movementState == TOTAL_STATES-1:
            # go up
            y = lengthTowerList[toTower]        
            z = heightTowerList[4]

            self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ]))
              
    def executeMovement(self, fromTower, toTower):
        self.moveBlock(fromTower, toTower)

        self.moveArm()

    def run(self):
        while not self.isSolved():
            if self.gameState == 0:
                fromTower, toTower = self.decideFromTo(0, 1)
            elif self.gameState == 1:
                fromTower, toTower = self.decideFromTo(0, 2)
            elif self.gameState == 2:
                fromTower, toTower = self.decideFromTo(1, 2)

            self.executeMovement(fromTower, toTower)

            if self.concluded:
                self.towers[toTower].append(self.towers[fromTower].pop())
                self.gameState += 1
                if self.gameState >= 3:
                    self.gameState = 0
                self.concluded = False
                print(self.towers)

    def decideFromTo(self, one, other):
        if len(self.towers[one]) == 0:
            # self.towers[one].append(self.towers[other].pop())
            return other, one

        elif len(self.towers[other]) == 0:
            # self.towers[other].append(self.towers[one].pop())
            return one, other

        elif self.towers[one][-1] < self.towers[other][-1]:
            # self.towers[other].append(self.towers[one].pop())
            return one, other

        else:
            # self.towers[one].append(self.towers[other].pop())
            return other, one


    def isSolved(self):
        if len(self.towers[0]) == 0 and len(self.towers[1]) == 0 and len(self.towers[2]) == 4:
            return True
        else:
            return False
        

        
        
        
