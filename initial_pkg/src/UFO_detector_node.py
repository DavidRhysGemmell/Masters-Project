#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Float32

class UFO_detector:
    def __init__(self):
        rospy.init_node('UFO_detector_node', anonymous=True)    
        self.sub = rospy.Subscriber('/scan', LaserScan, self.detector)
        self.distance_pub = rospy.Publisher('/UFO_distance', Float64, queue_size=10)
        self.angle_pub = rospy.Publisher('/UFO_angle', Float32, queue_size=1)
        rospy.loginfo("UFO Detector node is active")
    def detector(self, LaserMsg):
        laser_scan_array=LaserMsg.ranges
        width_in_scans=0
        for i in range(719): #not accounting for 720->0 yet
                
                if laser_scan_array[i]-laser_scan_array[i+1] > 0.5: #object begin
                    object_begin=i
                    j=i
                    if j==719:
                        j=0
                    print('object began at', object_begin)
                    while laser_scan_array[j]-laser_scan_array[j+1] >=-0.5:
                        width_in_scans=width_in_scans+1
                        j=j+1
                        if j==719:
                            j=0
                    
                    object_end = j
                    print('object_end', object_end)
                    object_width= width_in_scans
                    object_middle_scan=object_begin+object_width/2
                    if object_middle_scan>720:
                        object_middle_scan=object_middle_scan-720
                    object_middle= 360-object_middle_scan/2
                    if object_middle >180:
                        object_middle = object_middle-360

                    
                    print('object_middle',object_middle_scan)
                    distance=laser_scan_array[int(object_middle_scan)]
                    print(distance)
                    self.distance_pub.publish(distance)
                    print(width_in_scans)
                    self.angle_pub.publish(object_middle) #real angle in deg, /scan message is filled anti-clockwise. consusing eh.


    
if __name__ == '__main__':
    
    rospy.loginfo('Code is running')
    UFO_detector()
    rospy.spin()