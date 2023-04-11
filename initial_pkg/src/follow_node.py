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



    def euler_from_quarternion(self, odom_msg):
        (roll, pitch, yaw) = euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])
        self.robot_position = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw)
    
    

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


      
    def predicted_movement(self): #take 1 second
        self.time_since_object_detected = rospy.get_rostime() - self.object_detected_time
        self.predicted_position=self.object_position + self.object_velocity * time_since_object_detected          





    def goal(self):
        self.goal= self.object_position + self.object_velocity #1 second ahead






    def main_loop(self):
        rospy.spin()


if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()
