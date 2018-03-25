#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from negomo_enshu.msg import NegomoTask
from negomo_enshu.srv import PlannerDefaultInteractionCall, PlannerDefaultInteractionCallResponse

def taskInputCb(req):
    print 'interactionCb'
    speechpub.publish(phrase) # speak phrase
    time.sleep(speaktime) # wait till speech finish
    return PlannerDefaultInteractionCallResponse(True, NegomoTask('task2',[''],['']))

if __name__ == '__main__':
    rospy.init_node('example_interaction_cb')
    phrase = rospy.get_param('~phrase', False)
    speaktime = rospy.get_param('~time', False)

    rospy.Service('/negomo/interactivewi/defaultinteraction', PlannerDefaultInteractionCall, taskInputCb)
    speechpub = rospy.Publisher('/speech', String, queue_size=10)

    rospy.spin()
