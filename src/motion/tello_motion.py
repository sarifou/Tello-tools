#!/usr/bin/env python3

import rospy
import time
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String


class Motion:

    def __init__(self):
        """
        Constructeur
        """
        self.pose = Pose()
        self.odomTopic = "/tello/odom"
        self.takeoffTopic = "/tello/takeoff"
        self.landTopic = "/tello/land"
        self.cmdVelTopic = "/tello/cmd_vel"
        self.pub_cmdVel = rospy.Publisher(self.cmdVelTopic, Twist, queue_size=1)
        self.pub_takeOff = rospy.Publisher(self.takeoffTopic, Empty, queue_size=1, latch=False)
        self.pub_land = rospy.Publisher(self.landTopic, Empty, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber(self.odomTopic, Odometry, self.odomCallback)
        time.sleep(1)

    def totakeoff(self):
        print(" To take off ")
        self.pub_takeOff.publish()
        time.sleep(3)

    
    def toland(self):
        print(" To land")
        self.pub_land.publish()

    def odomCallback(self, odom):
        self.pose = odom.pose.pose
        #print(self.pose.position.y)
        #print(odom.pose.pose)
        #rospy.sleep(3)
        """
        """
    def moveL(self, direction, speed, distance):
        """
        """
        velocity_msg = Twist()
        x0 = self.pose.position.x
        print("x0 :" +str(x0))
        y0 = self.pose.position.y
        print("y0 :" +str(y0))
        z0 = self.pose.position.z
        print("z0 :" +str(z0))
        if ( direction == "forward" ) :
            print("Forward move")
            velocity_msg.linear.y = abs(speed)
        elif ( direction == "backward") :
            print("Backward move")
            velocity_msg.linear.y = -abs(speed)
        elif ( direction == "right") :
            print("Right move") 
            velocity_msg.linear.x = abs(speed)
        elif (direction == "left") :
            print("Left move")
            velocity_msg.linear.x = -abs(speed)
        elif (direction == "up") :
            print("Up move")
            velocity_msg.linear.z = abs(speed)
        elif (direction == "down") :
            print("Down move") 
            velocity_msg.linear.z = -abs(speed)

        distance_moved = 0.0
        loop_rate = rospy.Rate(10)

        while True :
            rospy.loginfo("Mouvement linéaire du drone")
            #print("Velocity :" + str(velocity_msg.linear.y))
            self.pub_cmdVel.publish(velocity_msg)
            loop_rate.sleep()
            distance_moved = abs(math.sqrt((self.pose.position.x-x0)**2 + (self.pose.position.y-y0)**2 + (self.pose.position.z-z0)**2))
            print("Distance moved : "+ str(distance_moved))
            if not(distance_moved < distance) :
                rospy.loginfo("Position atteinte")
                break
        velocity_msg.linear.y=0
        self.pub_cmdVel.publish(velocity_msg)

def main():
    rospy.init_node("tello_motion", anonymous=True)
    motion = Motion()
    motion.totakeoff()
    time.sleep(2)
    #rospy.sleep(10)
    motion.moveL("left", 0.5, 0.5)
    time.sleep(1)
    motion.moveL("right", 0.5, 0.5)
    time.sleep(1)
    motion.toland()
    rospy.spin()

if __name__ == '__main__':
    try :
        print("Node")
        main()
    except rospy.ROSInterruptException:
        pass