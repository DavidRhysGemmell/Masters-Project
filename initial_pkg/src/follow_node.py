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
        
    def euler_from_quarternion(self, odom_msg):
         linear=odom_msg.pose.pose.position
         quaternion_orientation = odom_msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([quaternion_orientation.x, quaternion_orientation.y, quaternion_orientation.z, quaternion_orientation.w])
    
        if yaw < 0:
            real_yaw = (yaw*(180/math.pi))+360
        else:
            real_yaw = yaw*(180/math.pi) #angle of robot relative to world frame     
        print(f"x is {linear.x:.2f}, y is {linear.y:.2f}, z is {linear.z:.2f}, Yaw is {real_yaw:.2f}")
    
    def moving_object(self,moving_object_data):
        self.object_position = moving_object_data.position
        self.object_velocity = moving_object_data.velocity
        self.distance_to_object = moving_object_data.closest_distance
        self.angle_to_object = moving_object_data.angle_for_closest_distance

    def object_acceleration(self):

    def predicted_movement(self):
        predicted_path=self.object_position + self.object_velocity * time




    def main_loop(self):
        rospy.spin()


if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()
