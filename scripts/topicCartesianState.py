import sys
import rospy
import roslib
# from std_msgs.msg import String
import threading
# from gazebo_msgs.srv import GetLinkState 
# from gazebo_msgs.srv import GetModelState
# from gazebo_msgs.srv import SetLinkState 
# from gazebo_msgs.srv import SetModelState
# from gazebo_msgs.msg import LinkState # For getting information about link states
# import time
from std_msgs.msg import Float64

"""
    Description:    Thread that publishes a Float64 in a topic, when allowed by the main thread.
"""

class topicCartesianState(threading.Thread):
    def __init__ (self, pubTopic, mutex, flag):
        threading.Thread.__init__(self)   
        self.publisher = pubTopic
        self.rate = 1
        
        self.mutex = mutex
        self.flag = flag

        self.pub = rospy.Publisher(self.publisher, Float64, queue_size=10)
       
        self.value = 0
    
    # Main thread sets the value that the thread must publish in the topic
    def setValue(self, value):
        self.value = Float64(value)
        #print (self.value)

    # The main thread sets if the thread can publish or not
    def setFlag(self):
        self.flag = True

    def run(self):
        self.r = rospy.Rate(self.rate) # 10hz
        while not rospy.is_shutdown():  
            self.mutex.acquire()
            if self.flag == True:                       
                self.pub.publish(self.value)
                #self.r.sleep()
                #print (self.value)

                self.flag = False

                self.mutex.notify_all()   
            else:
                self.mutex.wait()
            
            self.mutex.release() 

            