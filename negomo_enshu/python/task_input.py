#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from negomo.msg import NegomoTask
from negomo.srv import PlannerDefaultInteractionCall, PlannerDefaultInteractionCallResponse

LINE_AA = 16

class Key:
    def __init__(self, _key, _xs, _ys, _xe, _ye):
        self.key = _key
        self.xs = _xs
        self.ys = _ys
        self.xe = _xe
        self.ye = _ye

class Node:
    def __init__(self, _id, _ys, _ye, _t_id):
        self.name = _id
        self.ys = _ys
        self.ye = _ye
        self.t_id = _t_id

class TextNode:
    def __init__(self, _id, _val, _ys, _ye, _e_id):
        self.name = _id
        self.value = _val
        self.ys = _ys
        self.ye = _ye
        self.e_id = _e_id

__subwinname__ = 'taskinput'
__keyboard__ = 'keyboard'
__color__ = (200, 200, 200)

subwinphase_ = 0
createtasksub_ = False
destroytasksub_ = False
taskboxes_ = []
taskentities_ = []
entitiesmat_ = []
taskname_ = ''
img_ = None

keyboxes_ = [Key('',0.0,0.0,0.5,0.2),Key('***',0.5,0.0,0.6,0.2),Key('del',0.6,0.0,0.7,0.2),Key('ce',0.7,0.0,0.8,0.2),Key('enter',0.8,0.0,1.0,0.2),
             Key('1',0.0,0.2,0.1,0.4),Key('2',0.1,0.2,0.2,0.4),Key('3',0.2,0.2,0.3,0.4),Key('4',0.3,0.2,0.4,0.4),Key('5',0.4,0.2,0.5,0.4),Key('6',0.5,0.2,0.6,0.4),Key('7',0.6,0.2,0.7,0.4),Key('8',0.7,0.2,0.8,0.4),Key('9',0.8,0.2,0.9,0.4),Key('0',0.9,0.2,1.0,0.4),
             Key('Q',0.0,0.4,0.1,0.6),Key('W',0.1,0.4,0.2,0.6),Key('E',0.2,0.4,0.3,0.6),Key('R',0.3,0.4,0.4,0.6),Key('T',0.4,0.4,0.5,0.6),Key('Y',0.5,0.4,0.6,0.6),Key('U',0.6,0.4,0.7,0.6),Key('I',0.7,0.4,0.8,0.6),Key('O',0.8,0.4,0.9,0.6),Key('P',0.9,0.4,1.0,0.6),
             Key('A',0.05,0.6,0.15,0.8),Key('S',0.15,0.6,0.25,0.8),Key('D',0.25,0.6,0.35,0.8),Key('F',0.35,0.6,0.45,0.8),Key('G',0.45,0.6,0.55,0.8),Key('H',0.55,0.6,0.65,0.8),Key('J',0.65,0.6,0.75,0.8),Key('K',0.75,0.6,0.85,0.8),Key('L',0.85,0.6,0.95,0.8),
             Key('Z',0.1,0.8,0.2,1.0),Key('X',0.2,0.8,0.3,1.0),Key('C',0.3,0.8,0.4,1.0),Key('V',0.4,0.8,0.5,1.0),Key('B',0.5,0.8,0.6,1.0),Key('N',0.6,0.8,0.7,1.0),Key('M',0.7,0.8,0.8,1.0)]
target_box_ = 0
enable_keyboard_ = False

def subMouseCb(event, x, y, flags, param):
    global taskname_
    global ents_
    global vals_
    global taskentities_
    global subwinphase_
    global destroytasksub_
    global input_key_
    global target_box_
    global enable_keyboard_
    width=480
    height=640
    if event == cv2.EVENT_LBUTTONDOWN:
        if subwinphase_ == 0:
            for tb in taskboxes_:
                if height*tb.ys <= y and y <= height*tb.ye:
                    subwinphase_ = 1
                    taskname_ = tb.name
                    # go into entity
                    img = np.zeros((height, width, 3), dtype=np.uint8)
                    img[:] = (255, 255, 255)
                    taskentities_ = entitiesmat_[tb.t_id]
                    for eb in entitiesmat_[tb.t_id]:
                        boxth = 2
                        cv2.rectangle(img, ((boxth >> 1), int(height*eb.ys)+(boxth >> 1)), (((width-boxth) >> 1), int(height*eb.ye)-(boxth >> 1)), __color__, boxth)
                        cv2.rectangle(img, (((width-boxth) >> 1), int(height*eb.ys)+(boxth >> 1)), (width-(boxth >> 1), int(height*eb.ye)-(boxth >> 1)), __color__, boxth)
                        cv2.putText(img, eb.name, (10, int(height*eb.ys)+30), cv2.FONT_HERSHEY_SIMPLEX, 1, __color__, 1, LINE_AA)
                        cv2.putText(img, eb.value, ((width >> 1) + 10, int(height*eb.ys)+30), cv2.FONT_HERSHEY_SIMPLEX, 1, __color__, 1, LINE_AA)
                    cv2.imshow(__subwinname__, img)
                    break
        elif subwinphase_ == 1:
            for tb in taskentities_:
                if height*tb.ys <= y and y <= height*tb.ye:
                    if tb.name == 'enter':
                        ents_ = []
                        vals_ = []
                        for eb in taskentities_:
                            if eb.name == 'enter':
                                continue
                            ents_ += [eb.name]
                            vals_ += [eb.value]
                        subwinphase_ = 2
                        destroytasksub_ = True
                    else:
                        target_box_ = tb.e_id
                        keyboxes_[0].key = tb.value
                        enable_keyboard_ = True
                        createKeyboard()
                    break

def createKeyboard():
    width=1000
    height=250
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = (255, 255, 255)
    for bb in keyboxes_:
        boxth = 2
        cv2.rectangle(img, (int(width*bb.xs)+(boxth >> 1), int(height*bb.ys)+(boxth >> 1)), (int(width*bb.xe)-(boxth >> 1), int(height*bb.ye)-(boxth >> 1)), __color__, boxth)
        cv2.putText(img, bb.key, (int(width*bb.xs)+10, int(height*bb.ys)+30), cv2.FONT_HERSHEY_SIMPLEX, 1, __color__, 1, LINE_AA)
    cv2.imshow(__keyboard__, img)

def keyMouseCb(event, x, y, flags, param):
    global keyboxes_
    global enable_keyboard_
    global taskentities_
    width=1000
    height=250
    if not enable_keyboard_:
        return
    if event == cv2.EVENT_LBUTTONDOWN:
        for kb in keyboxes_:
            if width*kb.xs <= x and x <= width*kb.xe and height*kb.ys <= y and y <= height*kb.ye:
                if kb.key == 'enter':
                    taskentities_[target_box_].value = keyboxes_[0].key
                    enable_keyboard_ = False
                    width=480
                    height=640
                    img = np.zeros((height, width, 3), dtype=np.uint8)
                    img[:] = (255, 255, 255)
                    for eb in taskentities_:
                        boxth = 2
                        cv2.rectangle(img, ((boxth >> 1), int(height*eb.ys)+(boxth >> 1)), (((width-boxth) >> 1), int(height*eb.ye)-(boxth >> 1)), __color__, boxth)
                        cv2.rectangle(img, (((width-boxth) >> 1), int(height*eb.ys)+(boxth >> 1)), (width-(boxth >> 1), int(height*eb.ye)-(boxth >> 1)), __color__, boxth)
                        cv2.putText(img, eb.name, (10, int(height*eb.ys)+30), cv2.FONT_HERSHEY_SIMPLEX, 1, __color__, 1, LINE_AA)
                        cv2.putText(img, eb.value, ((width >> 1) + 10, int(height*eb.ys)+30), cv2.FONT_HERSHEY_SIMPLEX, 1, __color__, 1, LINE_AA)
                    cv2.imshow(__subwinname__, img)
                elif kb.key == 'del':
                    keyboxes_[0].key = keyboxes_[0].key[:-1]
                elif kb.key == 'ce':
                    keyboxes_[0].key = ''
                else:
                    keyboxes_[0].key += kb.key
                createKeyboard()
                break

def taskInputCb(req):
    global img_
    global taskboxes_
    global entitiesmat_
    global subwinphase_
    global createtasksub_
    if req.warn:
        return PlannerDefaultInteractionCallResponse(True, NegomoTask('',[],[]))
    subwinphase_ = 0
    width=480
    height=640
    img_ = np.zeros((height, width, 3), dtype=np.uint8)
    img_[:] = (255, 255, 255)
    taskboxes_ = []
    taskentities_ = []
    ys = 0.0
    dy = 0.1
    t_id = 0
    for tb in req.tasks:
        ye = ys + dy
        taskboxes_ += [Node(tb.task_name, ys, ye, t_id)]
        ents = []
        e_ys = 0.0
        e_dy = 0.1
        e_id = 0
        for e in range(len(tb.entity_names)):
            ents += [TextNode(tb.entity_names[e], tb.entity_values[e], e_ys, e_ys+e_dy, e_id)]
            e_ys += e_dy
            e_id += 1
        ents += [TextNode('enter', '', e_ys, e_ys+e_dy, e_id)]
        entitiesmat_ += [ents]
        boxth = 2
        cv2.rectangle(img_, ((boxth >> 1), int(height*ys)+(boxth >> 1)), (width-(boxth >> 1), int(height*ye)-(boxth >> 1)), __color__, boxth)
        cv2.putText(img_, tb.task_name, (10, int(height*ys)+30), cv2.FONT_HERSHEY_SIMPLEX, 1, __color__, 1, LINE_AA)
        ys += dy
        t_id += 1
    createtasksub_ = True
    while not destroytasksub_:
        pass # wait for input finish
    return PlannerDefaultInteractionCallResponse(True, NegomoTask(taskname_,ents_,vals_))

def run():
    global createtasksub_
    global destroytasksub_
    if createtasksub_:
        cv2.imshow(__subwinname__, img_)
        cv2.namedWindow(__subwinname__, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(__subwinname__, subMouseCb)
        createKeyboard()
        cv2.namedWindow(__keyboard__, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(__keyboard__, keyMouseCb)
        createtasksub_ = False
    elif destroytasksub_:
        cv2.destroyWindow(__subwinname__)
        cv2.destroyWindow(__keyboard__)
        destroytasksub_ = False
