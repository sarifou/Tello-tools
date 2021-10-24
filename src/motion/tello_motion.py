#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String



class Motion:

    def __init__(self):
        """
        Constructeur
        """
        self.odomTopic = "/tello/odom"
        self.takeoffTopic = "/tello/takeoff"
        self.landTopic = "/tello/land"
        #self.odomSub = rospy.Subscriber(self.odomTopic, Odometry, self.odomCallback)
        self.takeoffPub = rospy.Publisher(self.takeoffTopic, Empty, queue_size=1, latch=False)
        self.landPub = rospy.Publisher(self.landTopic, Empty, queue_size=1, latch=False)
        time.sleep(1)

    def totakeoff(self):
        print(" To take off ")
        self.takeoffPub.publish()
        time.sleep(3)

    
    def toland(self):
        print(" To land")
        self.landPub.publish()

    def odomCallback(self, odom):
        #print(odom.pose.pose)
        #rospy.sleep(3)
        """
        """

def main():
    rospy.init_node("tello_motion", anonymous=True)
    motion = Motion()
    motion.totakeoff()
    rospy.sleep(10)
    motion.toland()
    rospy.spin()

if __name__ == '__main__':
    try :
        print("Node")
        main()
    except rospy.ROSInterruptException:
        pass