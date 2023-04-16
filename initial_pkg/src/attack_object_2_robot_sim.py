#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import Twist

class Attack_node:
    def __init__(self):
        rospy.init_node('Attack_node', anonymous=True)  
        rospy.loginfo("Attack node is active")
        self.dist_sub = rospy.Subscriber('/UFO_distance', Float64, self.distance_sub)
        self.ang_sub = rospy.Subscriber('/UFO_angle', Float32, self.angle_sub)
        self.pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
        self.vel=Twist()
        self.ctrl_c=False
        rospy.on_shutdown(self.shutdown) 
        self.angle_vel=0
        self.linear_vel=0


    def distance_sub(self,distance):
        self.ufo_distance=distance.data
        #print(self.ufo_distance)
    def angle_sub(self,angle):
        self.ufo_angle=angle.data
        #print(self.ufo_angle)
        self.attack()
    def attack(self):
        if abs(self.ufo_angle)>1:
            self.angle_vel= -3*self.ufo_angle/180
            self.vel.angular.z=self.angle_vel
            #print(self.angle_vel)
            # if angle_vel>0:
                #print("turning left")
            # elif angle_vel<0:
                #print("turning right")
        if abs(self.ufo_angle>=90):
            self.linear_vel=0
        else:
            self.linear_vel = (1-abs(self.angle_vel)/3)/2   

        self.vel.linear.x=self.linear_vel
        self.pub.publish(self.vel)

    def shutdown(self):
        self.vel.angular.z=0
        self.vel.linear.x=0
        self.pub.publish(self.vel)
        print("shutting down")

        

if __name__ == '__main__':
    
    rospy.loginfo('Code is running')
    Attack_node()
    rospy.spin()