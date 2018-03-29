#!/usr/bin/env python

import cv2
import rospy
import numpy as np
import time
import yaml
from collections import namedtuple
from std_msgs.msg import String
from negomo_enshu.msg import NegomoTask
from negomo_enshu.srv import PlannerDefaultInteractionCall, PlannerDefaultInteractionCallResponse

Phrase = namedtuple('Phrase', 'phrase time template tail')
Result = namedtuple('Result', 'phrase time')
Choice = namedtuple('Choice', 'phrase time')

__timethreashold__ = 10.0 # speech listened within 10 seconds is a valid listen
phrases_ = {'': Phrase('', 0.0, '', '')}
choices_ = {'': Choice('', 0.0)}

result_ = []
def resultIn(msg):
    print 'resultIn %s' % msg.data
    global result_
    result_ += [Result(msg.data, time.time())]

def speak(phrase, tout, template):
    print 'speak %s, %f, %s' % (phrase, tout, template)
    if phrase != '':
        speechpub.publish(phrase) # speak phrase
    if template != '':
        templatepub.publish(template) # set answer choices
    time.sleep(tout) # wait till speech finish

def listen(tout):
    print 'listen %f' % tout
    listen_start = time.time()
    while True:
        if (time.time() - listen_start) > tout:
            return False
        elif len(result_) == 0:
            pass
        elif (time.time() - result_[-1].time) > __timethreashold__:
            pass
        else:
            return True

def confirm(choice):
    global result_
    print 'confirm %s' % choice.phrase
    taskname = result_[-1].phrase
    result_ = result_[:-1]
    if confirm_on:
        speak(phrases_['confirm'].phrase + choices_[choice.phrase].phrase + phrases_['confirm'].tail,
              phrases_['confirm'].time + choices_[choice.phrase].time, phrases_['confirm'].template)
        yes = True
        if listen(5.0) and result_[-1].phrase != 'yes':
            yes = False
            result_ = result_[:-1]
        if yes:
            speak(choices_['yes'].phrase, choices_['yes'].time, '')
            return taskname
        else:
            speak(choices_['no'].phrase, choices_['no'].time, '')
            conversation()
    else:
        speechpub.publish(phrases_['confirm'].phrase + choices_[choice.phrase].phrase + phrases_['confirm'].tail)
        time.sleep(phrases_['confirm'].time + choices_[choice.phrase].time)
        return taskname

def conversation():
    print 'conversation'
    if len(result_) > 0:
        if (time.time() - result_[-1].time) < __timethreashold__:
            return confirm(result_[-1])
    speak(phrases_['react'].phrase, phrases_['react'].time, phrases_['react'].template)
    if not listen(15.0):
        return ''
    return confirm(result_[-1])

def taskInputCb(req):
    print 'taskInputCb'
    if req.warn:
        speak(phrases_['refuse'].phrase, phrases_['refuse'].time, phrases_['refuse'].template)
        return PlannerDefaultInteractionCallResponse(True, NegomoTask('',[],[]))
    t_ents = {'': ['']}
    t_vals = {'': ['']}
    for tb in req.tasks:
        # copy ents but replace person
        ents = []
        vals = []
        for e in range(len(tb.entity_names)):
            ents += [tb.entity_names[e]]
            if tb.entity_names[e] == 'person':
                val = '***'
            else:
                val = tb.entity_values[e]
            vals += [val]
        t_ents[tb.task_name] = ents
        t_vals[tb.task_name] = vals
    # listen for task
    try:
        taskname = conversation()
    except Exception, exc:
        taskname = ''
        print exc
    return PlannerDefaultInteractionCallResponse(True, NegomoTask(taskname,t_ents[taskname],t_vals[taskname]))


if __name__ == '__main__':
    rospy.init_node('negomo_speech_input')
    fname = rospy.get_param('~file', '')
    confirm_on = rospy.get_param('~confirm', False)

    rospy.Service('/negomo/interactivewi/defaultinteraction', PlannerDefaultInteractionCall, taskInputCb)
    speechpub = rospy.Publisher('/speech', String, queue_size=10)
    templatepub = rospy.Publisher('/settings/speech', String, queue_size=10)
    rospy.Subscriber('/detected/speech/template', String, resultIn)

    # example yaml file
    # see linux_kinect/tools/speechfile_example.yaml

    # read yaml
    stream = open(fname, 'r')
    docs = yaml.load_all(stream)
    for i, doc in enumerate(docs):
        r = {}
        for k, v in doc.items():
            r[k] = v
        if r['tag'] == 'choice':
            choices_[r['key']] = Choice(r['phrase'], r['time'])
        else:
            phrases_[r['tag']] = Phrase(r['phrase'], r['time'], r['template'], r['tail'])

    rospy.spin()
