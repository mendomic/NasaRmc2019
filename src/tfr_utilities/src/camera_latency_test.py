#! /usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
import matplotlib.pyplot as plt
import time
class LatencyTracker(object):
    def __init__(self):
        self.times = []
        self.bridge = CvBridge()

        self.fig, self.axes = plt.subplots()
        self.axes.axis('off')

        self.set_black_display()
        self.state = "black"
        self.time_toggled = None
        self.record_time = None
        self.toggle_wait = 2
        self.average_latency_milli = 2
        self.count = 100;
        self.count_time = None

    def inc_count(self):
        self.count += 1
        if self.count >= 100:
            elapsed = rospy.get_rostime() - self.count_time
            rospy.loginfo("Frequency of calls: {} hz".format(self.count/elapsed.to_sec()))
            self.count = 0
            self.count_time = rospy.get_rostime()
            
          

    def set_black_display(self):
        self.fig.patch.set_facecolor((0,0,0))
        plt.draw()
        plt.pause(0.00000000000001)


    def set_white_display(self):
        self.fig.patch.set_facecolor((1,1,1))
        plt.draw()
        plt.pause(0.00000000000001)

    def get_intensity(self, cv_image):
        (rows,cols) = cv_image.shape
        offset = 10
        roi = np.array(cv_image[rows/2-offset:rows/2+offset, cols/2-offset:cols/2+offset])
        average = cv2.mean(roi)[0]
        rospy.loginfo("Intensity: {} ".format(average))
        return average

    def record(self, current_time):      
        elapsed_time = current_time-self.time_toggled
        ms = elapsed_time.to_sec()*1000
        fps = 1/elapsed_time.to_sec()
        ms = round(ms)
        rospy.loginfo("Latency: {} ms, {} fps".format(ms, fps))
        self.times.append(ms)
        #rospy.loginfo(self.times)
        self.average_latency_milli = sum(self.times)/len(self.times)
        rospy.loginfo("Average Latency: {} ms".format(self.average_latency_milli))

    def toggle(self):
        self.record_time = None
        if self.state == "black":
            self.state = "white"
            self.time_toggled = rospy.get_rostime()
            self.set_white_display()
        elif self.state == "white":
            self.state = "black"
            self.time_toggled = rospy.get_rostime()
            self.set_black_display()

    def on_image_get(self, image_data):
        if not self.time_toggled:
            self.toggle()
            self.count_time = rospy.get_rostime()
        self.inc_count()
        if self.record_time:
            if (rospy.get_rostime() - self.record_time).to_sec() >= self.toggle_wait:
                self.toggle()
            return
        current_time = rospy.get_rostime()
        cv_image = self.bridge.imgmsg_to_cv2(image_data, "mono8")
        intensity = self.get_intensity(cv_image)
        if self.state == "black":
            if intensity < 200:
                self.record(current_time)
                self.record_time = current_time
        elif self.state == "white":
            if intensity > 50:
                self.record(current_time)
                self.record_time = current_time

if __name__ == '__main__':
    tracker = LatencyTracker()
    rospy.init_node("camera_latency_tester")
    image_topic = rospy.get_param("~image_topic")
    mng = plt.get_current_fig_manager()
    mng.full_screen_toggle()
    image_sub = rospy.Subscriber(image_topic, Image, tracker.on_image_get, queue_size=1)
    #let this keep things running instead of spin() so that the gui thread can
    #run in the callback
    plt.show()
