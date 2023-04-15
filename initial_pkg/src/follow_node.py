
#! /usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from find_moving_objects.msg import MovingObject
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Follow_node:
    def __init__(self):
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


        self.moving_object_sub = rospy.Subscriber('/moving_objects', MovingObject, self.moving_object)
        self.rgbd_odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.euler_from_quarternion)
        rospy.loginfo("Follower node active")





    def shutdown(self): # on shutdown stop
        self.vel.angular.z=0
        self.vel.linear.x=0
        self.pub.publish(self.vel)
        # plt.imshow(self.grid) #uncomment to show map when ctrl_c
        # plt.show()            #uncomment to show map when ctrl_c
        self.vel.angular.z=0
        self.vel.linear.x=0
        self.pub.publish(self.vel)
        print("shutting down")
        sys.exit()



    def euler_from_quarternion(self, odom_msg):
        (roll, pitch, yaw) = euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])
        if yaw < 0: #from -pi -> pi to 0 -> 2pi
            real_yaw = yaw + 2*pi


        self.robot_position = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, real_yaw)
        
    

    def moving_object(self,moving_object_data):
        self.object_position = moving_object_data.position
        self.object_velocity = moving_object_data.velocity
        self.distance_to_object = moving_object_data.closest_distance
        self.angle_to_object = moving_object_data.angle_for_closest_distance
        self.object_detected_time = rospy.get_rostime()



    def object_acceleration(self): #future work




    def object_lost(self):
        if self.time_since_object_detected > 10:
            search_object=true


      
    def predicted_movement(self): 
        self.time_since_object_detected = rospy.get_rostime() - self.object_detected_time
        self.predicted_position=self.object_position + self.object_velocity * time_since_object_detected          





    def goal(self):
        self.goal= self.object_position + self.object_velocity #1 second ahead
        angle_to_goal = arctan2((self.robot_position.x - self.goal.x)/(self.robot_position.y))
        angle_to_turn = angle_to_goal - self.robot_position.real_yaw
        distance_to_goal = sqrt(pow((self.object_position.x-self.robot_position.x),2)+pow((self.object_position.y-self.robot_position.y),2))
        if angle_to_turn<-0.01: #just over 5 degrees left
            self.vel.angular.z = 0.5 
            print("turning left")
        elif angle_to_turn>0.01: #just over 5 degrees right
            self.vel.angular.z = -0.5
            print("turning right")
        else:
            print("On target")

        if distance_to_goal>0.02:
            self.vel.linear.x = 0.5
        else:
            self.vel.linear.x = 0.0
            print ("goal")









    def main_loop(self):
        rospy.spin()


if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()
