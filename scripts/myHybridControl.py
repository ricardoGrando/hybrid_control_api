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
import getpass

from sensor_msgs.msg import JointState

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

# Service to call the jacobian/forward package
def call_ik_service(currJointPose, targetPosition, targetOrientation, deltaEnd):
    rospy.init_node('hybridControl', anonymous=True)
    rospy.wait_for_service('hybrid_control_api/hybrid')
    cjp, dp = create_msg(currJointPose, targetPosition, targetOrientation)
    calc_ik_function = rospy.ServiceProxy('hybrid_control_api/hybrid', hybrid)
    res = calc_ik_function(cjp,dp)
    
    # Brush the response
    j = 0
    k = 0
    jacobian = np.zeros(shape=(6,7))
    err = np.zeros(shape=(6,1))
    
    for i in range(0, len(res.targJointPose)):
        # Gets the jacobian matrix from the server. It is pushed back as a jointPose. It needs to be improved, but is working
        if i >= len(endEffectorPosition)+err.shape[0]:  
            jacobian[k][j] = res.targJointPose[i].value
            j += 1
            if j == jacobian.shape[1]:
                j = 0
                k += 1
            
            if k == jacobian.shape[0]:
                break
         
        elif i < err.shape[0]: 
            # Gets the err vector from the server. It is pushed back as a jointPose. It needs to be improved, but is working
            err[i][0] = res.targJointPose[i].value
        else: 
            # Gets the position and orientation of the end-effector
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

    # Defines the delta angles
    deltaAngles = np.dot(inverseJacobian, deltaEndEuler)
       
    return deltaAngles  

class hybridControl(object):
    def __init__(self):
        np.set_printoptions(suppress=True) 
    
        # The initial position of the angles and its end effector cartesian position
        self.endEffectorInitialPosition = np.array([ 0.435, 0.15, 0.2, 1.0, 0.0, 0.0, 0.0 ])
        self.angles = np.array([-1.270851663074207,0.7706552679983241,1.1857071812176423,0.690658380033005,1.076204383634198,-1.2691182991333085,-1.0401750075045482])
        
        # The current joint pose is the initial pose
        self.currJointPose = [  ('l_arm_sh_p1', self.angles[0]),\
                                ('l_arm_sh_r', self.angles[1]),\
                                ('l_arm_sh_p2', self.angles[2]),\
                                ('l_arm_el_y', self.angles[3]),\
                                ('l_arm_wr_r', self.angles[4]),\
                                ('l_arm_wr_y', self.angles[5]),\
                                ('l_arm_wr_p', self.angles[6])
                            ]
        # List of threads and the mutex to control the threads with the runOnThormang class
        self.linkThreads = []
        self.mutex = threading.Condition()        
        
        # Call ros service to define the end-effector
        call_ik_service(self.currJointPose, self.endEffectorInitialPosition[0:3], self.endEffectorInitialPosition[3:7], np.zeros(shape=(7,)))
        #print(endEffectorPosition)

        #time.sleep(100)
        # Creates the thread for publishing in each joint
        for i in range(0, len(pubList)):
            self.linkThreads.append(topicCartesianState(pubList[i], self.mutex, False))

        # Start them   
        for i in range(0, len(pubList)):
	        #self.linkThreads[i].daemon = True
            self.linkThreads[i].start()

        self.thormangPublishers = runOnThormang()
        
        # Load the model
        self.model = load_model("/home/"+getpass.getuser()+"/catkin_ws/src/hybrid_control_api/scripts/model__340_2048_Adam_sigmoid_150_120_90_75_65_55_7_10000000_0.013154525557590856_0.01312057321594821_0.0007109376021267358.h5")

        # Its limits for normalization/desnormalization
        self.highestList = np.array([   0.0996578,  0.09700995, 0.09993384, 0.33062578, 0.28312842, 0.31182562,\
                                        0.27920513, 1.59999821, 1.59999794, 1.59999969, 1.29999984, 2.79999662,\
                                        1.3999988 , 1.3999999 , 0.08726649, 0.08726648, 0.0872665,  0.0872665,\
                                        0.08726646, 0.0872665,  0.08726644])

        self.lowestList = np.array([    -0.09980583, -0.09704175, -0.09984278, -0.31763874, -0.28009778, -0.31739168,\
                                        -0.28857708, -1.59999977, -1.59999941, -1.59999798, -1.29999983, -2.79999956,\
                                        -1.39999721, -1.39999988, -0.08726647, -0.08726649, -0.0872665,  -0.08726644,\
                                        -0.08726649, -0.08726646, -0.08726647])

        # The first real position is the initial position
        self.realEndEffector = self.endEffectorInitialPosition
        # First desired position
        self.desiredPosition = np.array([ x, lengthTowerList[0], heightTowerList[-1], 1.0, 0.0, 0.0, 0.0 ])
        # Vector of the distance between the desired and the real position
        self.delta = self.desiredPosition-self.endEffectorInitialPosition
        # Gripper closing value to be set in its joint
        self.gripper = 0
        # The pieces position in the hanoi game(Piece 4, 3, 2, 1. 4 is the biggest)
        self.towers = [[4, 3, 2, 1], [], []]
        # State of the arm movement to move the piece from a place A to a place B
        self.movementState = 0
        # Flag to set that the movement is completed
        self.concluded = False

        # Save the angles through the process
        csvfile = open("angles.csv", 'wb')
        self.csvwriter = csv.writer(csvfile)

        # The state of the Game. It changes when a movement is finished
        self.gameState = 0

        # Runs for the first time to put the arm at the desired initial position. It is runned 10 times because of the delay
        for i in range(0, 10):
            self.thormangPublishers.runOnThormang(self.angles, self.mutex, self.linkThreads, self.gripper, PUB_TIMES)  

    # This function limits the vector between the actual position of the end effector and the desired position
    # The delta(vector) is devided by two until the biggest value of the vector gets smaller than a MAX_STEP
    # Is done by getting the biggest value of the vector because the end-effector orientation is fixed and it only moves in a unique dimension at once
    # It is applied for the jacobian and for the ANN 
    def limitVector(self):
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
    # Moves the arm using Jacobian or ANN, depending on the arriving threshold. For position is used the norm and for orientation is the w distance
    def moveArm(self):

        # Gets the end-effector delta
        self.limitVector()

        # Verify the distance towards the desired position and orientation
        if np.linalg.norm((self.desiredPosition-self.realEndEffector)[0:3]) > ARRIVED_POS_THRESHOLD or ((self.desiredPosition-self.realEndEffector)[-1]) > ARRIVED_ORI_THRESHOLD:
            # uses the ANN
            if np.linalg.norm((self.desiredPosition-self.realEndEffector)[0:3]) > NETORJACOBIAN_THRESHOLD:
                # Set the input of the ANN to be predicted(delta end and angles)
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
                # Normalize the predict array
                predictArray = normalizeArray(predictArray, self.highestList[:14], self.lowestList[:14])
                # The ANN output
                deltaAnglesPredicted = (np.array(self.model.predict(predictArray.reshape(1,14)))[0])  
                # Desnormalize the predicted angles
                deltaAnglesPredicted = desnormalizeArray(deltaAnglesPredicted, self.highestList[14:], self.lowestList[14:])
                # The angle plus the delta angles
                self.angles += deltaAnglesPredicted.reshape(1,7)[0] 
                # Runs on thormang
                self.thormangPublishers.runOnThormang(self.angles, self.mutex, self.linkThreads, self.gripper, PUB_TIMES)           
                
                # Publishes on the /robotis/set_joint_states too
                jointStateValue = JointState()
                jointStateValue.name = ['l_arm_sh_p1', 'l_arm_sh_r', 'l_arm_sh_p2', 'l_arm_el_y', 'l_arm_wr_r', 'l_arm_wr_y', 'l_arm_wr_p']
                jointStateValue.position = [self.angles[0], self.angles[1], self.angles[2], self.angles[3], self.angles[4], self.angles[5], self.angles[6]]
                #print(jointStateValue)
                jointStatePublisher = rospy.Publisher("/robotis/set_joint_states", JointState, queue_size=10)
                jointStatePublisher.publish(jointStateValue)

                # Saves the angles
                self.csvwriter.writerow(self.angles.tolist())

                # Sets the current pose of the arm
                self.currJointPose = [  ('l_arm_sh_p1', self.angles[0]),\
                                ('l_arm_sh_r', self.angles[1]),\
                                ('l_arm_sh_p2', self.angles[2]),\
                                ('l_arm_el_y', self.angles[3]),\
                                ('l_arm_wr_r', self.angles[4]),\
                                ('l_arm_wr_y', self.angles[5]),\
                                ('l_arm_wr_p', self.angles[6])
                            ]
                # And calls the server to define the end-effector
                call_ik_service(self.currJointPose, self.desiredPosition[0:3], self.desiredPosition[3:7], np.zeros(shape=(7,)))
                # The new realEndEffector is then set
                self.realEndEffector = np.array([endEffectorPosition[0:][0][1], endEffectorPosition[1:][0][1], \
                                                    endEffectorPosition[2:][0][1], endEffectorPosition[3:][0][1], \
                                                    endEffectorPosition[4:][0][1], endEffectorPosition[5:][0][1], \
                                                    endEffectorPosition[6:][0][1]])

                print("Net")
                print("Norm total: "+str(np.linalg.norm((self.desiredPosition-self.realEndEffector))))
                print("Norm pos: "+str(np.linalg.norm((self.desiredPosition-self.realEndEffector)[0:3])))
                print("Norm ori: "+str(np.linalg.norm((self.desiredPosition-self.realEndEffector)[3:])))

                print(self.realEndEffector)

            # Uses the Jacobian
            else:
                # Defines the pose of the arm
                self.currJointPose = [  ('l_arm_sh_p1', self.angles[0]),\
                                ('l_arm_sh_r', self.angles[1]),\
                                ('l_arm_sh_p2', self.angles[2]),\
                                ('l_arm_el_y', self.angles[3]),\
                                ('l_arm_wr_r', self.angles[4]),\
                                ('l_arm_wr_y', self.angles[5]),\
                                ('l_arm_wr_p', self.angles[6])
                            ]
                # Calls the service to calc the Jacobian Matrix based on the current pose. The matrix is inverted and dotted with the delta, setting the new angle increment
                self.angles += call_ik_service(self.currJointPose, self.desiredPosition[0:3], self.desiredPosition[3:7], self.delta)

                # Send to the topic when it gets close
                self.thormangPublishers.runOnThormang(self.angles, self.mutex, self.linkThreads, self.gripper, PUB_TIMES)
                
                # Publishes on the /robotis/set_joint_states too
                jointStateValue = JointState()
                jointStateValue.name = ['l_arm_sh_p1', 'l_arm_sh_r', 'l_arm_sh_p2', 'l_arm_el_y', 'l_arm_wr_r', 'l_arm_wr_y', 'l_arm_wr_p']
                jointStateValue.position = [self.angles[0], self.angles[1], self.angles[2], self.angles[3], self.angles[4], self.angles[5], self.angles[6]]
                #print(jointStateValue)
                jointStatePublisher = rospy.Publisher("/robotis/set_joint_states", JointState, queue_size=10)
                jointStatePublisher.publish(jointStateValue)

                print("Jacobian")
                print("Norm total: "+str(np.linalg.norm((self.desiredPosition-self.realEndEffector))))
                print("Norm pos: "+str(np.linalg.norm((self.desiredPosition-self.realEndEffector)[0:3])))
                print("Norm ori: "+str(np.linalg.norm((self.desiredPosition-self.realEndEffector)[3:])))
                
                # Saves the angles
                self.csvwriter.writerow(self.angles.tolist())

                # Defines the current joint pose again
                self.currJointPose = [  ('l_arm_sh_p1', self.angles[0]),\
                                ('l_arm_sh_r', self.angles[1]),\
                                ('l_arm_sh_p2', self.angles[2]),\
                                ('l_arm_el_y', self.angles[3]),\
                                ('l_arm_wr_r', self.angles[4]),\
                                ('l_arm_wr_y', self.angles[5]),\
                                ('l_arm_wr_p', self.angles[6])
                            ]
                
                # And calls the service once again
                call_ik_service(self.currJointPose, self.desiredPosition[0:3], self.desiredPosition[3:7], self.delta)

                # To define the new real end effector position
                self.realEndEffector = np.array([endEffectorPosition[0:][0][1], endEffectorPosition[1:][0][1], \
                                                    endEffectorPosition[2:][0][1], endEffectorPosition[3:][0][1], \
                                                    endEffectorPosition[4:][0][1], endEffectorPosition[5:][0][1], \
                                                    endEffectorPosition[6:][0][1]])
               
        else:
            # Movement concluded
            if (self.movementState != 2 and self.movementState != 6 and self.movementState != 7):  
                if self.movementState == TOTAL_STATES-1:
                    self.movementState = 0
                    self.concluded = True                
                else:     
                    self.movementState += 1
            # Or publish on thormang
            else:
                self.thormangPublishers.runOnThormang(self.angles, self.mutex, self.linkThreads, self.gripper, PUB_TIMES)           

            #print("State of the Movement:" + str(self.movementState))

    # Sets a new target catesian position
    def setTarget(self, target):

        self.desiredPosition = target
        self.delta = (self.desiredPosition-self.realEndEffector)

    # Machine State to move a block from a tower A to B
    def moveBlock(self, fromTower, toTower):   
        if self.movementState == 0:
            # Go above of the desired tower to take the block
            y = lengthTowerList[fromTower]        
            z = heightTowerList[4]

            self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ]))

        elif self.movementState == 1:
            # Go down to grab the block
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
            # Go up
            y = lengthTowerList[fromTower]        
            z = heightTowerList[4]            

            self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ])) 


        elif self.movementState == 4:
            # Go to above of the tower we want to place the block
            y = lengthTowerList[toTower]        
            z = heightTowerList[4]

            self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ]))

        elif self.movementState == 5:
            # Go down
            y = lengthTowerList[toTower]  

            z = heightTowerList[len(self.towers[toTower])]
            self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ]))

            self.waitForGripper = 0

        elif self.movementState == 6:
            # Release the block and wait            

            if self.waitForGripper <= GRIPPER_WAIT_MAX_TIME:
                self.waitForGripper += 1
            else:
                #self.waitForGripper = 0
                self.movementState += 1

                self.gripper = 0

                self.waitForGripper = 0

        elif self.movementState == 7:
            # Wait and set to go up
            if self.waitForGripper <= GRIPPER_WAIT_MAX_TIME:
                self.waitForGripper += 1
            else:
                #self.waitForGripper = 0
                self.movementState += 1

                y = lengthTowerList[toTower]        
                z = heightTowerList[4]

                self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ]))

        elif self.movementState == TOTAL_STATES-1:
            # Go up
            y = lengthTowerList[toTower]        
            z = heightTowerList[4]

            self.setTarget(np.array([ x, y, z, 1.0, 0.0, 0.0, 0.0 ]))
              
    # Execute the movement
    def executeMovement(self, fromTower, toTower):
        self.moveBlock(fromTower, toTower)

        self.moveArm()

    # Hanoi Tower Game's algorithm for pair number of blocks
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

    # Decide from which tower and to what tower the movement must be executed
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

    # verify if the game is finally solved
    def isSolved(self):
        if len(self.towers[0]) == 0 and len(self.towers[1]) == 0 and len(self.towers[2]) == 4:
            return True
        else:
            return False
        

        
        
        
