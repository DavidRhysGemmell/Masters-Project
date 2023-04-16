#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Float32, Bool

class UFO_detector:
    def __init__(self):
        rospy.init_node('UFO_detector_node', anonymous=True)    
        self.sub = rospy.Subscriber('/scan', LaserScan, self.detector)
        self.distance_pub = rospy.Publisher('/UFO_distance', Float64, queue_size=10)
        self.angle_pub = rospy.Publisher('/UFO_angle', Float32, queue_size=10)
        self.detector_pub=rospy.Publisher('/UFO_detected', Bool, queue_size=10)
        rospy.loginfo("UFO Detector node is active")
        self.bank_size = 40 #how many messages to determine relative object velocity, put hz of /scan
        self.distance_bank = []
        self.angle_bank = []

    def detector(self, LaserMsg):
        laser_scan_array=LaserMsg.ranges
        width_in_scans=0
        UFO_detected = False
        for i in range(719): #not accounting for 720->0 yet
                
                if laser_scan_array[i]-laser_scan_array[i+1] > 0.5: #object begin
                    object_begin=i
                    j=i
                    if j==719:
                        j=0
                    #print('object began at', object_begin)
                    while laser_scan_array[j]-laser_scan_array[j+1] >=-0.5: #until object end
                        width_in_scans=width_in_scans+1
                        j=j+1
                        if j==719:
                            j=0
                    UFO_detected=True
                    object_end = j
                    #print('object_end', object_end)
                    object_width= width_in_scans
                    object_middle_scan=object_begin+object_width/2
                    if object_middle_scan>720:
                        object_middle_scan=object_middle_scan-720
                    self.object_middle= 360-object_middle_scan/2
                    if self.object_middle >180:
                        self.object_middle = self.object_middle-360

                    rospy.loginfo("Object detected")
                    #print('object_middle',object_middle_scan)
                    self.distance=laser_scan_array[int(object_middle_scan)]
                    #print(distance)
                    self.distance_pub.publish(self.distance)
                    #print(width_in_scans)
                    self.angle_pub.publish(self.object_middle) #real angle in deg, /scan message is filled anti-clockwise. consusing eh.
                    self.velocity()
        #print('UFO?',UFO_detected)
        self.detector_pub.publish(UFO_detected)
    def velocity(self):
        for i in range (1,self.bank_size):
            self.distance_bank[i]=self.distance_bank[i-1] # move all distance values up one in the list, removing the earliest scan.
            self.angle_bank[i]=self.angle_bank[i-1]
        self.distance_bank[0] = self.distance
        self.angle_bank[0] = self.object_middle




    
if __name__ == '__main__':
    
    rospy.loginfo('Code is running')
    UFO_detector()
    rospy.spin()