#! /usr/bin/env python3
import rospy
import time

class Attack_node:
    def __init__(self):
        rospy.init_node('Attack_node', anonymous=True)  
        rospy.loginfo("Initialised")
        self.t=0
        self.detected_sub()

    def detected_sub(self):
        rospy.loginfo("Subject integration initialised")
        time.sleep(2)
        rospy.loginfo("Subject integration successful")
        for i in range (20):
            time.sleep(1)
            rospy.logwarn("WARNING: Integration is unstable")
        t=0
        while t<1:
            rospy.logerr("Integration Failed. Subject is alive.")
            rospy.logerr("Shutdown initiated.")
            rospy.logerr("Shutdown failed. Control is lost.")
   


if __name__ == '__main__':
    
    rospy.loginfo('Code is running')
    Attack_node()
    rospy.spin()