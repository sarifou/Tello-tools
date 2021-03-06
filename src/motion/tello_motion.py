#!/usr/bin/env python3

import rospy
import time
import math
import matplotlib.pyplot as plt
import numpy as np
import actionlib
from tello_tools.msg import MoveAction, MoveFeedback, MoveResult
from simple_pid import PID
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String

consigne = []
entree = []
sortie = []
draw = False


class Motion:

    def __init__(self):
        """
        Constructeur
        """
        self.kp = 1
        self.ki = 1
        self.kd = 1
        self.pose = Pose()
        self.velocity = Twist()
        self.feedback = MoveFeedback()
        self.fig, self.ax = plt.subplots()
        self.result = MoveResult()
        self.odomTopic = "/tello/odom"
        self.takeoffTopic = "/tello/takeoff"
        self.landTopic = "/tello/land"
        self.cmdVelTopic = "/tello/cmd_vel"
        self.move_action = actionlib.SimpleActionServer("basic_move", MoveAction, execute_cb=self.moveL, auto_start=False)
        self.move_action.start()
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
        self.velocity = odom.twist.twist
        #print(self.pose.position.y)
        #print(self.velocity)
        #rospy.sleep(3)
        """
        """
    def moveL(self, goal):
        """
        """
        rospy.loginfo(goal)
        success = True
        velocity_msg = Twist()
        x0 = self.pose.position.x
        #print("x0 :" +str(x0))
        y0 = self.pose.position.y
        #print("y0 :" +str(y0))
        z0 = self.pose.position.z
        #print("z0 :" +str(z0))
        if (goal.direction == "forward" ) :
            print("Forward move")
            velocity_msg.linear.y = abs(goal.speed)
        elif (goal.direction == "backward") :
            print("Backward move")
            velocity_msg.linear.y = -abs(goal.speed)
        elif (goal.direction == "right") :
            print("Right move") 
            velocity_msg.linear.x = abs(goal.speed)
        elif (goal.direction == "left") :
            print("Left move")
            velocity_msg.linear.x = -abs(goal.speed)
        elif (goal.direction == "up") :
            print("Up move")
            velocity_msg.linear.z = abs(goal.speed)
        elif (goal.direction == "down") :
            print("Down move") 
            velocity_msg.linear.z = -abs(goal.speed)

        distance_moved = 0.0
        loop_rate = rospy.Rate(10)

        while True :
            rospy.loginfo("Mouvement lin??aire du drone")
            if self.move_action.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            #print("Velocity :" + str(velocity_msg.linear.y))
            consigne.append(velocity_msg.linear.y)
            velocity_pid = self.pid(velocity_msg)
            entree.append(velocity_pid.linear.y)
            sortie.append(self.velocity.linear.y)
            self.pub_cmdVel.publish(velocity_msg)
            loop_rate.sleep()
            distance_moved = abs(math.sqrt((self.pose.position.x-x0)**2 + (self.pose.position.y-y0)**2 + (self.pose.position.z-z0)**2))
            self.feedback.distance_moved = distance_moved
            self.move_action.publish_feedback(self.feedback)
            print("Distance moved : "+ str(distance_moved))
            if not(distance_moved < goal.distance) :
                rospy.loginfo("Position atteinte")
                break
        rospy.loginfo(self.pose)
        time = range(len(consigne))
        self.ax.plot(time, consigne, c="blue", label="consigne")
        self.ax.plot(time, entree, c="red", label="entree pid")
        self.ax.plot(time, sortie, c="black", label="odom")
        plt.savefig("diag.png")
        velocity_msg.linear.x=0
        velocity_msg.linear.y=0
        velocity_msg.linear.z=0
        self.pub_cmdVel.publish(velocity_msg)
        if success :
            draw = True
            print("Draw")
            self.result.status = "Position atteinte"
            self.result.pose = self.pose
            self.move_action.set_succeeded(self.result)

    def pid(self, setPoint) :
        """
        """
        output = Twist()
        pid_vx = PID(self.kp, self.ki, self.kd, setPoint.linear.x)
        pid_vy = PID(self.kp, self.ki, self.kd, setPoint.linear.y)
        pid_vz = PID(self.kp, self.ki, self.kd, setPoint.linear.z)
        pid_wx = PID(self.kp, self.ki, self.kd, setPoint.angular.x)
        pid_wy = PID(self.kp, self.ki, self.kd, setPoint.angular.y)
        pid_wz = PID(self.kp, self.ki, self.kd, setPoint.angular.z)

        output.linear.x = pid_vx(self.velocity.linear.x)
        output.linear.y = pid_vy(self.velocity.linear.y)
        output.linear.z = pid_vz(self.velocity.linear.z)
        output.angular.x = pid_wx(self.velocity.angular.x)
        output.angular.y = pid_wy(self.velocity.angular.y)
        output.angular.z = pid_wz(self.velocity.angular.z)

        return output

def main():
    rospy.init_node("tello_motion", anonymous=True)
    motion = Motion()
    motion.totakeoff()  
    rospy.spin()

if __name__ == '__main__':
    try :
        print("Node")
        main()
    except rospy.ROSInterruptException:
        pass