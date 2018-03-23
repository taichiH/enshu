#!/usr/bin/env/python

import cv2
import numpy as np
from std_msgs.msg import String
from threading import Lock
import copy

LINE_AA = 16

__width__ = 480
__height__ = 350

__pending__ = (128, 128, 128)
__tasknow__ = (78, 205, 5)

__rady__ = 24
__radx__ = 24

__mid__ = (__width__ >> 1)
__left__ = (__width__ >> 1) - (__radx__ << 1) - 10
__right__ = (__width__ >> 1) + (__radx__ << 1) + 10

__level0__ = __rady__ + 5
__level1__ = __rady__*3 + 15
__level2__ = __rady__*5 + 25
__level3__ = __rady__*7 + 35
__level4__ = __rady__*9 + 45
__level5__ = __rady__*11 + 55

class Node:
    def __init__(self, name, x, y, msg):
        self.color = __pending__
        self.x = x
        self.y = y
        self.name = name
        self.msg = msg

class Connection:
    def __init__(self, nodes, msg1, msg2):
        self.color = __pending__
        for n in nodes:
            if n.msg == msg1:
                node1 = n
            elif n.msg == msg2:
                node2 = n
        self.xs = node1.x
        self.xe = node2.x
        self.ys = node1.y + __rady__
        self.ye = node2.y - __rady__

nodes_ = {'free': [], 'busy': []}
connections_ = {'free': [], 'busy': []}

# create nodes
nodes_['free'] += [Node('break', __mid__, __level0__, 'break')]
nodes_['free'] += [Node('done', __right__, __level0__, '-> done')]
nodes_['free'] += [Node('no', __left__, __level1__, 'break -> no')]
nodes_['free'] += [Node(':|', __right__, __level1__, 'break -> reset')]
nodes_['free'] += [Node('yes', __mid__, __level1__, 'break -> yes')]
nodes_['free'] += [Node('listen', __mid__, __level2__, 'during')]
nodes_['free'] += [Node('lost', __left__, __level3__, 'during -> escape')]
nodes_['free'] += [Node('lost?', __right__, __level3__, 'during -> redo')]
nodes_['free'] += [Node('done', __mid__, __level4__, 'during -> finish')]
nodes_['free'] += [Node(':|', __mid__, __level5__, 'break -> yes -> reset')]
connections_['free'] += [Connection(nodes_['free'], 'break', 'break -> no')]
connections_['free'] += [Connection(nodes_['free'], 'break', 'break -> reset')]
connections_['free'] += [Connection(nodes_['free'], 'break', 'break -> yes')]
connections_['free'] += [Connection(nodes_['free'], 'break -> yes', 'during')]
connections_['free'] += [Connection(nodes_['free'], 'during', 'during -> escape')]
connections_['free'] += [Connection(nodes_['free'], 'during', 'during -> redo')]
connections_['free'] += [Connection(nodes_['free'], 'during -> escape', 'during -> finish')]
connections_['free'] += [Connection(nodes_['free'], 'during', 'during -> finish')]
connections_['free'] += [Connection(nodes_['free'], 'during -> finish', 'break -> yes -> reset')]

nodes_['busy'] += [Node('check', __mid__, __level0__, 'check')]
nodes_['busy'] += [Node('done', __right__, __level0__, '-> done')]
nodes_['busy'] += [Node('no', __left__, __level1__, 'check -> no')]
nodes_['busy'] += [Node('warn', __right__, __level1__, 'check -> warn')]
nodes_['busy'] += [Node('done', __right__, __level2__, 'check -> warn -> finish')]
nodes_['busy'] += [Node('yes', __mid__, __level1__, 'check -> yes')]
nodes_['busy'] += [Node('listen', __mid__, __level2__, 'during')]
# nodes_['busy'] += [Node('lost', __left__, __level3__, 'during -> escape')]
# nodes_['busy'] += [Node('lost?', __right__, __level3__, 'during -> redo')]
# nodes_['busy'] += [Node('done', __mid__, __level4__, 'during -> finish')]
# nodes_['busy'] += [Node(':|', __mid__, __level5__, 'check -> yes -> reset')]
nodes_['busy'] += [Node('done', __mid__, __level3__, 'during -> finish')]
nodes_['busy'] += [Node(':|', __mid__, __level4__, 'check -> yes -> reset')]
connections_['busy'] += [Connection(nodes_['busy'], 'check', 'check -> no')]
connections_['busy'] += [Connection(nodes_['busy'], 'check', 'check -> warn')]
connections_['busy'] += [Connection(nodes_['busy'], 'check -> warn', 'check -> warn -> finish')]
connections_['busy'] += [Connection(nodes_['busy'], 'check', 'check -> yes')]
connections_['busy'] += [Connection(nodes_['busy'], 'check -> yes', 'during')]
# connections_['busy'] += [Connection(nodes_['busy'], 'during', 'during -> escape')]
# connections_['busy'] += [Connection(nodes_['busy'], 'during', 'during -> redo')]
# connections_['busy'] += [Connection(nodes_['busy'], 'during -> escape', 'during -> finish')]
connections_['busy'] += [Connection(nodes_['busy'], 'during', 'during -> finish')]
connections_['busy'] += [Connection(nodes_['busy'], 'during -> finish', 'check -> yes -> reset')]

curnodes_ = 'free'
nodesimg_ = None
drawnodes_ = False
imglock_ = Lock()
def interactionCb(msg):
    global curnodes_
    global nodesimg_
    global drawnodes_
    if 'start' in msg.data:
        drawnodes_ = True
        curnodes_ = msg.data.split(':')[-1]
    elif msg.data == 'finish':
        drawnodes_ = False
        return
    nodes = copy.deepcopy(nodes_[curnodes_])
    connections = connections_[curnodes_][:]
    for i in range(len(nodes)):
        if nodes[i].msg == msg.data:
            nodes[i].color = __tasknow__
            break
    # create image
    imglock_.acquire()
    nodesimg_ = np.zeros((__height__, __width__, 3), dtype=np.uint8)
    nodesimg_[:] = (255, 255, 255)
    # draw nodes
    for n in nodes:
        cv2.ellipse(nodesimg_, (n.x, n.y), (__radx__, __rady__), 0.0, 0, 360, n.color, 1, LINE_AA)
        cv2.putText(nodesimg_, n.name, (n.x-__radx__+5, n.y-__rady__+30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, n.color, 1, LINE_AA)
    for c in connections:
        cv2.line(nodesimg_, (c.xs,c.ys), (c.xe,c.ye), c.color, 1)
    imglock_.release()

