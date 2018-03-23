#!/usr/bin/env python

import rospy
from std_msgs.msg import *

__print__ = ''

def Print(msg):
    global __print__

    target = int(msg.data.split(';')[0]) * 2
    if target == 0:
        __print__ = msg.data.split(';')[-1] + '\n' + '\n'.join(__print__.split('\n')[1:])
    else:
        __print__ = '\n'.join(__print__.split('\n')[0:target]) + '\n' + msg.data.split(';')[-1] + '\n' + '\n'.join(__print__.split('\n')[target+1:])
    print __print__

if __name__ == '__main__':
    rospy.init_node('negomo_print')
    num = rospy.get_param('~max_targets')

    rospy.Subscriber('/negomo/observation_sequence', String, Print)

    __print__ = 'null\n\n' * num

    rospy.spin()
