#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from negomo_enshu.msg import NegomoTask
from negomo_enshu.srv import PlannerDefaultInteractionCall, PlannerDefaultInteractionCallResponse

def taskInputCb(req):
    print 'interactionCb'
    if req.warn:
        speechpub.publish(phrase_refuse)
        time.sleep(speaktime_refuse)
        return PlannerDefaultInteractionCallResponse(True, NegomoTask('',[''],['']))
    speechpub.publish(phrase) # speak phrase
    time.sleep(speaktime) # wait till speech finish
    return PlannerDefaultInteractionCallResponse(True, NegomoTask(taskto,[''],['']))

if __name__ == '__main__':
    rospy.init_node('example_interaction_cb')
    taskto = rospy.get_param('~taskto', 'task0')
    phrase = rospy.get_param('~phrase', 'none')
    speaktime = rospy.get_param('~time', 0.0)
    phrase_refuse = rospy.get_param('~phrase_refuse', 'none')
    speaktime_refuse = rospy.get_param('~time_refuse', 0.0)

    rospy.Service('/negomo/interactivewi/defaultinteraction', PlannerDefaultInteractionCall, taskInputCb)
    speechpub = rospy.Publisher('/speech', String, queue_size=10)

    rospy.spin()
