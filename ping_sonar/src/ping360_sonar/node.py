#!/usr/bin/env python

from math import cos, pi, sin

import numpy as np
import rospy
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from ping_sonar.cfg import sonarConfig
from ping360_sonar.sensor import Ping360
from ping_sonar.srv import sendingSonarConfig

from ping_sonar.msg import SonarEcho2

# Global Variables

device = None
baudrate = None
gain = None
numberOfSamples = None
transmitFrequency = None
sonarRange = None
speedOfSound = None
step = None
imgSize = None
queue_size = None
threshold = None
debug = None
transmitDuration = None
samplePeriod = None
updated = False
firstRequest = True
enableImageTopic = False
enableScanTopic = False
enableDataTopic = False
maxAngle = None
minAngle = None
oscillate = None


def callback(config, level):
    global updated, gain, numberOfSamples, transmitFrequency, transmitDuration, sonarRange, \
        speedOfSound, samplePeriod, debug, step, imgSize, queue_size, threshold, firstRequest
    if not firstRequest:  # Avoid overiting params set in the launch file
        rospy.loginfo("Reconfigure Request")
        # Update Ping 360 Parameters
        gain = config['gain']
        numberOfSamples = config['numberOfSamples']
        transmitFrequency = config['transmitFrequency']
        sonarRange = config['range']
        speedOfSound = config['speedOfSound']
        samplePeriod = calculateSamplePeriod(
            sonarRange, numberOfSamples, speedOfSound)
        transmitDuration = adjustTransmitDuration(
            sonarRange, samplePeriod, speedOfSound)
        debug = config['debug']
        step = config['step']
        queue_size = config['queueSize']
        threshold = config['threshold']
        updated = True
    firstRequest = False
    return config

def handeSonarConfigService(req):
    global updated, gain, numberOfSamples, transmitFrequency, transmitDuration, sonarRange, \
        speedOfSound, samplePeriod, debug, step, imgSize, queue_size, threshold, firstRequest
    rospy.loginfo("Reconfigure Request")
    # Update Ping 360 Parameters
    # gain = config['gain']
    # numberOfSamples = config['numberOfSamples']
    # transmitFrequency = config['transmitFrequency']
    sonarRange = req.range
    step = req.stepSize
    samplePeriod = calculateSamplePeriod(
        sonarRange, numberOfSamples, speedOfSound)
    transmitDuration = adjustTransmitDuration(
        sonarRange, samplePeriod, speedOfSound)
    # debug = config['debug']
    # step = config['step']
    # queue_size = config['queueSize']
    # threshold = config['threshold']
    updated = True
    return True

def main():
    global updated, gain, numberOfSamples, transmitFrequency, transmitDuration, sonarRange, \
        speedOfSound, samplePeriod, debug, step, imgSize, queue_size, threshold, \
        enableDataTopic, enableImageTopic, enableScanTopic, oscillate

    # Initialize node
    rospy.init_node('ping360_node')

    # Ping 360 Parameters
    device = rospy.get_param('~device', "/dev/ttyUSB0")
    baudrate = rospy.get_param('~baudrate', 115200)
    gain = rospy.get_param('~gain', 0)
    numberOfSamples = rospy.get_param('~numberOfSamples', 200)  # Number of points
    transmitFrequency = rospy.get_param(
        '~transmitFrequency', 740)  # Default frequency
    sonarRange = rospy.get_param('~sonarRange', 1)  # in m
    speedOfSound = rospy.get_param('~speedOfSound', 1500)  # in m/s
    samplePeriod = calculateSamplePeriod(sonarRange, numberOfSamples, speedOfSound)
    transmitDuration = adjustTransmitDuration(
        sonarRange, samplePeriod, speedOfSound)
    debug = rospy.get_param('~debug', True)
    threshold = int(rospy.get_param('~threshold', 200))  # 0-255

    enableImageTopic = rospy.get_param('~enableImageTopic', True)
    enableScanTopic = rospy.get_param('~enableScanTopic', True)
    enableDataTopic = rospy.get_param('~enableDataTopic', True)

    maxAngle = int(rospy.get_param('~maxAngle', 400))  # 0-400
    minAngle = int(rospy.get_param('~minAngle', 0))  # 0-400
    FOV = maxAngle - minAngle  # The sonars field of view
    oscillate = int(rospy.get_param('~oscillate', True))
    sign = 1

    # Output and ROS parameters
    step = int(rospy.get_param('~step', 1))
    imgSize = int(rospy.get_param('~imgSize', 500))
    queue_size = int(rospy.get_param('~queueSize', 1))

    # # TODO: improve configuration validation
    # if FOV <= 0:
    #     rospy.logerr(
    #         """
    #         minAngle should be inferior to the maxAngle!
    #         Current settings:
    #             minAngle: {} - maxAngle: {}""".format(maxAngle, minAngle))
    #     rospy.signal_shutdown("Bad minAngle & maxAngle values")
    #     return

    if step >= FOV:
        rospy.logerr(
            """
            The configured step is bigger then the set FOV (maxAngle - minAngle)
            Current settings:
                step: {} - minAngle: {} - maxAngle: {} - FOV: {}""".format(step,
                                                                           maxAngle,
                                                                           minAngle,
                                                                           FOV))
        rospy.signal_shutdown("Bad minAngle & maxAngle or step values")
        return

    # Initialize sensor
    sensor = Ping360(device, baudrate)
    while not sensor.initialize():
        print("Initialized Sensor: %s" % sensor.initialize())
        print("didnt work trying again in 1 second")
        time.sleep(2)
        sensor = Ping360(device, baudrate)
        if rospy.is_shutdown():
            exit()
    print("Initialized Sensor: %s" % sensor.initialize())
    # Dynamic reconfigure server and service server
    srv = Server(sonarConfig, callback)
    s = rospy.Service('changeParametersSonar', sendingSonarConfig, handeSonarConfigService)
    # Global Variables
    angle = minAngle
    bridge = CvBridge()

    # Topic publishers
    # imagePub = rospy.Publisher(
    #     "/ping360_node/sonar/images", Image, queue_size=queue_size)
    rawPub = rospy.Publisher("sonar/intensity",
                             SonarEcho2, queue_size=queue_size)
    laserPub = rospy.Publisher(
        "/sonar/scan", LaserScan, queue_size=queue_size)

    # Initialize and configure the sonar
    updateSonarConfig(sensor, gain, transmitFrequency,
                      transmitDuration, samplePeriod, numberOfSamples)

    # Create a new mono-channel image
    image = np.zeros((imgSize, imgSize, 1), np.uint8)


    # Center point coordinates
    center = (float(imgSize / 2), float(imgSize / 2))

    rate = rospy.Rate(100)  # 100hz

    # Initial the LaserScan Intensities & Ranges
    while not rospy.is_shutdown():
        ranges = []
        intensities = []

        # Update to the latest config data
        if updated:
            updateSonarConfig(sensor, gain, transmitFrequency,
                              transmitDuration, samplePeriod, numberOfSamples)
            angle = minAngle
        # Get sonar response
        data = getSonarData(sensor, angle)

        # Contruct and publish Sonar data msg
        if enableDataTopic:
            rawDataMsg = generateRawMsg(angle, data, gain, numberOfSamples, transmitFrequency, speedOfSound, sonarRange, step)
            rawPub.publish(rawDataMsg)

        # Prepare scan msg
        if enableScanTopic:
            # Get the first high intensity value
            for detectedIntensity in data:
                if detectedIntensity >= threshold:
                    detectedIndex = data.index(detectedIntensity)
                    # The index+1 represents the number of samples which then can be used to deduce the range
                    distance = calculateRange(
                        (1 + detectedIndex), samplePeriod, speedOfSound)
                    if distance > 0 and distance <= 1:
                        ranges.append(distance)
                        intensities.append(detectedIntensity)

                        # ranges.append(distance)
                        # intensities.append(detectedIntensity)
                        if debug:
                            print("Object at {} grad : {}m - {}%".format(angle,
                                                                         ranges[0],
                                                                         float(intensities[0] * 100 / 255)))
                        # break
            # Contruct and publish Sonar scan msg
            scanDataMsg = generateScanMsg(ranges, intensities, sonarRange, step, maxAngle, minAngle)
            laserPub.publish(scanDataMsg)

        '''

        # Prepare scan msg
        if enableScanTopic:
            # Get the first high intensity value
            for detectedIntensity in data:

                # if (detectedIntensity > 75):
                
                    # intensities.append(detectedIntensity)

                    # detectedIndex = data.index(detectedIntensity)
                    # distance = calculateRange(
                    #         (1 + detectedIndex), samplePeriod, speedOfSound)
                    # ranges.append(distance)

            
                if detectedIntensity >= threshold:
                    detectedIndex = data.index(detectedIntensity)
                    # The index+1 represents the number of samples which then can be used to deduce the range
                    distance = calculateRange(
                        (1 + detectedIndex), samplePeriod, speedOfSound)
                    # if distance >= 0.75 and distance <= sonarRange:
                    #     ranges.append(distance)
                    ranges.append(distance)
                    intensities.append(detectedIntensity)
                        # if debug:
                        #     print("Object at {} grad : {}m - {}%".format(angle,
                        #                                                  ranges[0],
                        #                                                  float(intensities[0] * 100 / 255)))
            
                # Contruct and publish Sonar scan msg
                scanDataMsg = generateScanMsg(ranges, intensities, sonarRange, step, maxAngle, minAngle)
                laserPub.publish(scanDataMsg)
        '''

        # Contruct and publish Sonar image msg
        # if enableImageTopic:
        #     linear_factor = float(len(data)) / float(center[0])
        #     try:
        #         for i in range(int(center[0])):
        #             if(i < center[0]):
        #                 pointColor = data[int(i * linear_factor - 1)]
        #             else:
        #                 pointColor = 0
        #             for k in np.linspace(0, step, 8 * step):
        #                 theta = 2 * pi * (angle + k) / 400.0
        #                 x = float(i) * cos(theta)
        #                 y = float(i) * sin(theta)
        #                 image[int(center[0] + x)][int(center[1] + y)
        #                                           ][0] = pointColor
        #     except IndexError:
        #         rospy.logwarn(
        #             "IndexError: data response was empty, skipping this iteration..")
        #         continue
        #
        #     publishImage(image, imagePub, bridge)




        # calculate next angle step
        angle += sign * step
        # print(angle)

        angle = wrap_angle(angle)

        if maxAngle > minAngle:
            if angle >= maxAngle:
                if not oscillate:
                    angle = minAngle
                else:
                    angle = maxAngle
                    sign = -1

            if angle <= minAngle and oscillate:
                sign = 1
                angle = minAngle
        else:
            if angle >= maxAngle and angle < minAngle:
                if not oscillate:
                    angle = minAngle
                    sign = 1
                else:
                    angle = maxAngle
                    sign = -1


        rate.sleep()


def getSonarData(sensor, angle):
    """
    Transmits the sonar angle and returns the sonar intensities
    Args:
        sensor (Ping360): Sensor class
        angle (int): Gradian Angle
    Returns:
        list: Intensities from 0 - 255
    """
    sensor.transmitAngle(angle)
    data = bytearray(getattr(sensor, '_data'))
    return [k for k in data]


def generateRawMsg(angle, data, gain, numberOfSamples, transmitFrequency, speedOfSound, sonarRange,stepSize):
    """
    Generates the raw message for the data topic
    Args:
        angle (int): Gradian Angle
        data (list): List of intensities
        gain (int)
        numberOfSamples (int)
        transmitFrequency (float)
        speedOfSound (int)
        sonarRange (int)
    Returns:
        SonarEcho: message
     """
    msg = SonarEcho2()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'sonar_frame'
    msg.angle = angle
    msg.gain = gain
    msg.number_of_samples = numberOfSamples
    msg.transmit_frequency = transmitFrequency
    msg.speed_of_sound = speedOfSound
    msg.range = sonarRange
    msg.step_size = stepSize
    msg.intensities = data
    return msg


def generateScanMsg(ranges, intensities, sonarRange, step, maxAngle, minAngle):
    """
    Generates the laserScan message for the scan topic
    Args:
        angle (int): Gradian Angle
        data (list): List of intensities
        sonarRange (int)
        step (int)
     """
    msg = LaserScan()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'sonar_frame'
    msg.angle_min = 2 * pi * minAngle / 400
    msg.angle_max = 2 * pi * maxAngle / 400
    msg.angle_increment = 2 * pi * step / 400
    msg.time_increment = 0
    msg.range_min = .75
    msg.range_max = sonarRange
    msg.ranges = ranges
    msg.intensities = intensities

    return msg


def publishImage(image, imagePub, bridge):
    try:
        imagePub.publish(bridge.cv2_to_imgmsg(image, "mono8"))
    except CvBridgeError as e:
        rospy.logwarn("Failed to publish sensor image")
        print(e)


def calculateRange(numberOfSamples, samplePeriod, speedOfSound, _samplePeriodTickDuration=25e-9):
    # type: (float, int, float, float) -> float
    """
      Calculate the range based in the duration
     """
    return numberOfSamples * speedOfSound * _samplePeriodTickDuration * samplePeriod / 2


def calculateSamplePeriod(distance, numberOfSamples, speedOfSound, _samplePeriodTickDuration=25e-9):
    # type: (float, int, int, float) -> float
    """
      Calculate the sample period based in the new range
     """
    return 2 * distance / (numberOfSamples * speedOfSound * _samplePeriodTickDuration)


def adjustTransmitDuration(distance, samplePeriod, speedOfSound, _firmwareMinTransmitDuration=5):
    # type: (float, float, int, int) -> float
    """
     @brief Adjust the transmit duration for a specific range
     Per firmware engineer:
     1. Starting point is TxPulse in usec = ((one-way range in metres) * 8000) / (Velocity of sound in metres
     per second)
     2. Then check that TxPulse is wide enough for currently selected sample interval in usec, i.e.,
          if TxPulse < (2.5 * sample interval) then TxPulse = (2.5 * sample interval)
        (transmit duration is microseconds, samplePeriod() is nanoseconds)
     3. Perform limit checking
     Returns:
        float: Transmit duration
     """
    duration = 8000 * distance / speedOfSound
    transmit_duration = max(
        2.5 * getSamplePeriod(samplePeriod) / 1000, duration)
    return max(_firmwareMinTransmitDuration, min(transmitDurationMax(samplePeriod), transmit_duration))


def transmitDurationMax(samplePeriod, _firmwareMaxTransmitDuration=500):
    # type: (float, int) -> float
    """
    @brief The maximum transmit duration that will be applied is limited internally by the
    firmware to prevent damage to the hardware
    The maximum transmit duration is equal to 64 * the sample period in microseconds

    Returns:
        float: The maximum transmit duration possible
    """
    return min(_firmwareMaxTransmitDuration, getSamplePeriod(samplePeriod) * 64e6)


def getSamplePeriod(samplePeriod, _samplePeriodTickDuration=25e-9):
    # type: (float, float) -> float
    """  Sample period in ns """
    return samplePeriod * _samplePeriodTickDuration


def updateSonarConfig(sensor, gain, transmitFrequency, transmitDuration, samplePeriod, numberOfSamples):
    global updated
    #print("worked1")
    sensor.set_gain_setting(gain)
    #print("worked2")
    sensor.set_transmit_frequency(transmitFrequency)
    #print("worked3")
    #print(transmitDuration)
    sensor.set_transmit_duration(int(transmitDuration))
    #print("worked4")
    sensor.set_sample_period(int(samplePeriod))
    #print("worked5")
    sensor.set_number_of_samples(numberOfSamples)
    #print("worked6")
    updated = False

def wrap_angle(angle):
    while angle < 0:
        angle += 400
    while angle >= 400:
        angle -= 400
    return angle
