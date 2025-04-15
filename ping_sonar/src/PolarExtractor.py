#! /usr/bin/env python3


from re import A, M
import rospy
from ping_sonar.msg import SonarEcho2, SonarRange
from std_msgs.msg import Int8MultiArray,MultiArrayLayout,MultiArrayDimension
import numpy as np

class PolarExtractor():

    def __init__(self):
        
        #published array
        self.range_array = []
        self.angle_array = []

        #Subscribers
        rospy.Subscriber('/sonar/intensity', SonarEcho2, self.polar_callback)

        #Publishers
        self.range_publisher = rospy.Publisher('/sonar/range', SonarRange, queue_size=1)


    def polar_callback(self, data):

        ##Collect data
        data_arr = data.intensities #Data from sonar
        #transmit_frequency = data.transmit_frequency
        samples_length= data.number_of_samples #length of data array * 1200 now
        max_distance = data.range #Sonar range * 10 m now
        angle = data.angle #Angle of individual beam
        step_size = data.step_size # Angle step size
        Final = SonarRange()
        
        MeterCells = samples_length/max_distance #Array length for readings within 1 m 

        arr = np.empty(max_distance,int) #Array for meter based measurement

        indexes = int(400/step_size)
        threshold = 45

        k = max_distance - 2
        rho = -1
        for x in range(indexes):
            ##Averaging for each meter reading 
            for i in range(max_distance):
                mlocation = int(i*MeterCells)
                sum = 0
                for j in range(-10,10):
                    sum += data_arr[mlocation + j]
                arr[i] = sum/20

            while (k > 0):
                if abs(arr[k] - arr[k+1]) > threshold:
                    rho = k + 1
                k = k-1

            if rho == -1:
                rospy.loginfo("No wall at " + str(angle))
            rospy.loginfo(str(rho))
            rospy.loginfo(str(angle))
            
        
        self.range_array.append(rho)
        self.angle_array.append(angle)

        Final.sonar_range = self.range_array
        Final.angle = self.angle_array

        if (len(self.range_array) >= indexes):
            self.range_publisher.publish(Final)
            # reset array
            self.range_array = [] 
            self.angle_array = []


        # Refresh = self.PolarPublisher(self.PubArr)

        # if Refresh == True:
        #     self.PubArr = []
                

    # def PolarPublisher(self,arr):

    #     if len(arr) == 360/self.step_size:
    #         self.Pub.publish(arr)
    #         ArrayFull = True
    #     else:
    #         ArrayFull = False
        
    #     return ArrayFull



if __name__ == "__main__":
    rospy.init_node("PolarExtractor")
    PolarExtractor()
    rospy.spin()