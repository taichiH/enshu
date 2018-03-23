#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess

def speak(msg):
    subprocess.call('echo "' + msg.data + '" | festival --tts', shell=True)
    tts_finished_publisher.publish('')

if __name__ == '__main__':
    rospy.init_node('simulated_speak')
    speech_subscriber = rospy.Subscriber('/speech', String, speak, queue_size=10)
    tts_finished_publisher = rospy.Publisher('/speech/finished', String, queue_size=10)

    rospy.spin()
