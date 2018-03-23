#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from threading import Lock
import copy
from collections import OrderedDict

LINE_AA = 16

__total_width__ = 1200
__total_height__ = 560

__w__ = 240
__h__ = 560

__pending__ = (128, 128, 128)
__tasknow__ = (78, 205, 5)
__taskdone__ = (204, 83, 56)

__rady__ = 24
__radx__ = 24

__mid__ = (__w__ >> 1)
__left__ = (__w__ >> 1) - (__radx__ << 1) - 10
__right__ = (__w__ >> 1) + (__radx__ << 1) + 10

__level0__ = __rady__ + 5
__level1__ = __rady__*3 + 15
__level2__ = __rady__*5 + 25
__level3__ = __rady__*7 + 35
__level4__ = __rady__*9 + 45
__level5__ = __rady__*11 + 55
__level6__ = __rady__*13 + 65
__level7__ = __rady__*15 + 75
__level8__ = __rady__*17 + 85


class Node:
    def __init__(self, name, x, y, msg, nodeEnd=False):
        self.color = __pending__
        self.x = x
        self.y = y
        self.name = name
        self.msg = msg
        self.nodeEnd = nodeEnd

class Connection:
    def __init__(self, nodes, msg1, msg2, style='down'):
        self.color = __pending__
        self.th = 1
        for i, n in enumerate(nodes):
            if n.msg == msg1:
                node1 = n
                self.n1 = i
            elif n.msg == msg2:
                node2 = n
                self.n2 = i
        if style == 'down':
            self.xs = node1.x
            self.xe = node2.x
            self.ys = node1.y + __rady__
            self.ye = node2.y - __rady__
        elif style == 'left':
            self.xs = node1.x - __radx__
            self.xe = node2.x + __radx__
            self.ys = node1.y
            self.ye = node2.y

nodes_ = {'run': [], 'call': []}
connections_ = {'run': [], 'call': []}
prevs_ = {'run': -1, 'call': []}

# create nodes
nodes_['run'] += [Node('main', __mid__, __level0__, 'run')]
nodes_['run'] += [Node('abort', __left__, __level1__, 'run -> abort', True)]
nodes_['run'] += [Node('thread', __mid__, __level1__, 'run -> threads')]
nodes_['run'] += [Node('update', __mid__, __level2__, 'run -> threads -> update')]
nodes_['run'] += [Node('finish', __mid__, __level3__, 'run -> threads -> update -> finished', True)]
connections_['run'] += [Connection(nodes_['run'], 'run', 'run -> abort')]
connections_['run'] += [Connection(nodes_['run'], 'run', 'run -> threads')]
connections_['run'] += [Connection(nodes_['run'], 'run -> threads', 'run -> threads -> update')]
connections_['run'] += [Connection(nodes_['run'], 'run -> threads -> update', 'run -> threads -> update -> finished')]

nodes_['call'] += [Node('call', __mid__, __level0__, 'call', True)]
nodes_['call'] += [Node('context', __mid__, __level1__, 'call -> context')]
nodes_['call'] += [Node('react', __left__, __level2__, 'call -> context -> reactive')]
nodes_['call'] += [Node('true?', __mid__, __level5__, 'call -> context -> reactive -> personp')]
nodes_['call'] += [Node('observe', __mid__, __level6__, 'call -> context -> observe')]
nodes_['call'] += [Node('else', __left__, __level5__, 'call -> context -> reactive -> personelse')]
nodes_['call'] += [Node('?', __left__, __level6__, 'call -> context -> reactive -> personelse -> preparing')]
nodes_['call'] += [Node('proact', __left__, __level7__, 'call -> context -> reactive -> preparing -> personr')]
nodes_['call'] += [Node('false', __mid__, __level7__, 'call -> context -> reactive -> personn')]
nodes_['call'] += [Node('proact', __right__, __level2__, 'call -> context -> proactive')]
nodes_['call'] += [Node('abort', __mid__, __level3__, 'call -> context -> proactive -> abort')]
nodes_['call'] += [Node('prepare', __mid__, __level4__, 'call -> context -> proactive -> prepare')]
nodes_['call'] += [Node('proact', __right__, __level6__, 'call -> context -> proactive -> usual')]
nodes_['call'] += [Node('finish', __mid__, __level8__, 'call -> context -> finish')]
connections_['call'] += [Connection(nodes_['call'], 'call', 'call -> context')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context', 'call -> context -> reactive')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context -> reactive', 'call -> context -> reactive -> personp')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context -> reactive -> personp', 'call -> context -> observe')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context -> reactive', 'call -> context -> reactive -> personelse')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context -> reactive -> personelse', 'call -> context -> reactive -> personn')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context -> reactive -> personelse', 'call -> context -> reactive -> personelse -> preparing')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context -> reactive -> personelse -> preparing', 'call -> context -> reactive -> preparing -> personr')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context -> reactive -> personelse -> preparing', 'call -> context -> reactive -> personn')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context', 'call -> context -> proactive')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context -> proactive', 'call -> context -> proactive -> abort')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context -> proactive', 'call -> context -> proactive -> prepare')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context -> proactive', 'call -> context -> proactive -> usual')]
connections_['call'] += [Connection(nodes_['call'], 'call -> context -> proactive -> usual', 'call -> context -> observe', 'left')]

def init(num):
    global nodes_
    global connections_
    for i in range(num):
        key = 'thread' + str(i)
        nodes_[key] = []
        connections_[key] = []
        prevs_[key] = -1
        nodes_[key] += [Node(str(i), __mid__, __level0__, key)]
        nodes_[key] += [Node('none', __left__, __level1__, key + ' -> noperson')]
        nodes_[key] += [Node('finish', __left__, __level2__, key + ' -> noperson -> finished', True)]
        nodes_[key] += [Node('elapsed', __right__, __level1__, key + ' -> timeupdate', True)]
        nodes_[key] += [Node('newobs', __mid__, __level1__, key + ' -> newobservation', True)]
        nodes_[key] += [Node('process', __mid__, __level3__, key + ' -> RunThread/ObserveFrom -> process')]
        nodes_[key] += [Node('nottarg', __left__, __level3__, key + ' -> process -> nottarget')]
        nodes_[key] += [Node('prep', __left__, __level4__, key + ' -> process -> npreparing')]
        nodes_[key] += [Node('conflict', __left__, __level5__, key + ' -> process -> conflict')]
        nodes_[key] += [Node('break', __mid__, __level5__, key + ' -> process -> break')]
        nodes_[key] += [Node('finish', __mid__, __level6__, key + ' -> finish')]
        nodes_[key] += [Node('observe', __right__, __level5__, key + ' -> RunThread/Process/TaskPlannerBridge -> observe')]
        nodes_[key] += [Node('fail', __left__, __level8__, key + ' -> observe -> actionfail', True)]
        nodes_[key] += [Node('none', __mid__, __level8__, key + ' -> observe -> none', True)]
        nodes_[key] += [Node('to_process', __right__, __level8__, key + ' -> observe -> finish')]
        connections_[key] += [Connection(nodes_[key], key, key + ' -> noperson')]
        connections_[key] += [Connection(nodes_[key], key + ' -> noperson', key + ' -> noperson -> finished')]
        connections_[key] += [Connection(nodes_[key], key, key + ' -> timeupdate')]
        connections_[key] += [Connection(nodes_[key], key + ' -> timeupdate', key + ' -> RunThread/Process/TaskPlannerBridge -> observe')]
        connections_[key] += [Connection(nodes_[key], key, key + ' -> newobservation')]
        connections_[key] += [Connection(nodes_[key], key + ' -> newobservation', key + ' -> RunThread/ObserveFrom -> process')]
        connections_[key] += [Connection(nodes_[key], key + ' -> RunThread/ObserveFrom -> process', key + ' -> process -> nottarget', 'left')]
        connections_[key] += [Connection(nodes_[key], key + ' -> process -> nottarget', key + ' -> finish')]
        connections_[key] += [Connection(nodes_[key], key + ' -> RunThread/ObserveFrom -> process', key + ' -> process -> npreparing')]
        connections_[key] += [Connection(nodes_[key], key + ' -> process -> npreparing', key + ' -> finish')]
        connections_[key] += [Connection(nodes_[key], key + ' -> RunThread/ObserveFrom -> process', key + ' -> process -> conflict')]
        connections_[key] += [Connection(nodes_[key], key + ' -> process -> conflict', key + ' -> finish')]
        connections_[key] += [Connection(nodes_[key], key + ' -> RunThread/ObserveFrom -> process', key + ' -> process -> break')]
        connections_[key] += [Connection(nodes_[key], key + ' -> RunThread/ObserveFrom -> process', key + ' -> RunThread/Process/TaskPlannerBridge -> observe')]
        connections_[key] += [Connection(nodes_[key], key + ' -> RunThread/Process/TaskPlannerBridge -> observe', key + ' -> observe -> actionfail')]
        connections_[key] += [Connection(nodes_[key], key + ' -> RunThread/Process/TaskPlannerBridge -> observe', key + ' -> observe -> none')]
        connections_[key] += [Connection(nodes_[key], key + ' -> RunThread/Process/TaskPlannerBridge -> observe', key + ' -> observe -> finish')]
        connections_[key] += [Connection(nodes_[key], key + ' -> process -> break', key + ' -> finish')]

texts_ = OrderedDict([('negotiation_target', ''),
                      ('status.negotiating', ''),
                      ('status.state', ''),
                      ('search', ''),
                      ('candidates', '')])
textsLock_ = Lock()
def varCb(msg):
    global texts_
    textsLock_.acquire()
    msg = msg.data.split(':')
    if not msg[0] in texts_: # not valid variable
        return
    if msg[0][-1] == '+':
        texts_[msg[0][:-1]] += ',' + msg[1]
    else:
        texts_[msg[0]] = msg[1]
    textsLock_.release()

nodesLock_ = Lock()
def flowCb(msg):
    global nodes_
    global prevs_
    key = ''
    itm = 0
    finish = False
    nodesLock_.acquire()
    # check if reset
    if msg.data == 'reset-all':
        for k, v in nodes_.iteritems():
            prevs_[k] = -1
            for j in range(len(nodes_[k])):
                nodes_[k][j].color = __pending__
            for j in range(len(connections_[k])):
                connections_[k][j].color = __pending__
        nodesLock_.release()
        return
    # get node
    for k, v in nodes_.iteritems():
        breakFlag = False
        for j in range(len(v)):
            if nodes_[k][j].msg == msg.data:
                if nodes_[k][j].nodeEnd:
                    key = k
                    itm = j
                    finish = True
                else:
                    # update
                    if prevs_[k] >= 0:
                        nodes_[k][prevs_[k]].color = __taskdone__
                        for l, c in enumerate(connections_[k]):
                            if c.n1 == prevs_[k] and c.n2 == j:
                                connections_[k][l].color = __taskdone__
                                connections_[k][l].th = 2
                                break
                    nodes_[k][j].color = __tasknow__
                    prevs_[k] = j
                breakFlag = True
                break
        if breakFlag:
            break
    # clear colors if node is finish
    if finish:
        prevs_[k] = -1
        for j in range(len(nodes_[k])):
            nodes_[k][j].color = __pending__
        for j in range(len(connections_[k])):
            connections_[k][j].color = __pending__
            connections_[k][j].th = 1
        nodes_[k][itm].color = __taskdone__
        for l, c in enumerate(connections_[k]):
            if c.n1 == itm:
                connections_[k][l].color = __taskdone__
                connections_[k][l].th = 2
                break
    nodesLock_.release()

nodesimg_ = None
def drawNodes(nodes, connections, offsetX, offsetY):
    global nodesimg_
    # draw nodes
    for n in nodes:
        cv2.ellipse(nodesimg_, (offsetX + n.x, offsetY + n.y), (__radx__, __rady__), 0.0, 0, 360, n.color, 1, LINE_AA)
        cv2.putText(nodesimg_, n.name, (offsetX + n.x-__radx__+5, offsetY + n.y-__rady__+30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, n.color, 1, LINE_AA)
    for c in connections:
        cv2.line(nodesimg_, (offsetX+c.xs,offsetY+c.ys), (offsetX+c.xe,offsetY+c.ye), c.color, c.th)

def drawImg(num):
    global nodesimg_
    nodesimg_ = np.zeros((__total_height__, __total_width__, 3), dtype=np.uint8)
    nodesimg_[:] = (255, 255, 255)
    nodesLock_.acquire()
    drawNodes(nodes_['run'], connections_['run'][:], 0, 0)
    drawNodes(nodes_['call'], connections_['call'][:], __w__, 0)
    for i in range(num):
        drawNodes(nodes_['thread'+str(i)], connections_['thread'+str(i)][:], __w__*(i+2), 0)
    nodesLock_.release()
    textsLock_.acquire()
    x = 10
    y = 350
    for k, v in texts_.iteritems():
        cv2.putText(nodesimg_, k + ': ' + v, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (50, 50, 50), 1, LINE_AA)
        y += 20
    textsLock_.release()


if __name__ == '__main__':
    rospy.init_node('negomo_viewprocess')
    rospy.Subscriber('/negomo/vp/negomo/var', String, varCb, queue_size=10)
    rospy.Subscriber('/negomo/vp/negomo/flow', String, flowCb, queue_size=10)
    num = rospy.get_param('~max_targets')

    cv2.namedWindow('negomoprocess', cv2.WINDOW_AUTOSIZE)

    init(num)
    while not rospy.is_shutdown():
        drawImg(num)
        cv2.imshow('negomoprocess', nodesimg_)
        cv2.waitKey(1)
