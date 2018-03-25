#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from std_msgs.msg import Int32, String, Empty
from negomo_enshu.msg import VpActivate, VpConnect
from negomo_enshu.srv import PlannerDefaultInteractionCall
from threading import Lock
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import draw_interactionwi as wi
import task_input as tinput

LINE_AA = 16

class Node:
    def __init__(self, _id, _xs, _ys, _xe, _ye):
        self.actionid = _id
        self.xs = _xs
        self.ys = _ys
        self.xe = _xe
        self.ye = _ye

__maxtasks__ = 3
__width__ = 1500
__taskwidth__ = 1000
__height__ = 640

__resize__ = True

__done__ = (128, 128, 128)
__pending__ = (204, 83, 56)
__gray__ = (200, 200, 200)
__taskdone__ = (200, 200, 200)
__tasknow__ = (78, 205, 5)
__taskerror__ = (108, 108, 255)
__taskintrp__ = (108, 173, 255)
__taskto__ = (248, 113, 255)

__tempexp__ = 0 # temp in exception
__temptask__ = 1 # reverse temp at head of task action
__tempintrp__ = 2 # temp from interruption
__tempmove__ = 3 # extra temp added when escaping task
__temponmove__ = 4 # under extra temp when returning to task

mouse_active_ = False

class TaskBox:
    def __init__(self):
        self.active = -1 # -1: not active, 0: in queue, 1: active
        self.entid = -1 # used when activating from queue
        self.text = 'not active' # whether not-active, task, or queue
        self.actions = [] # list of actions in task
        self.curaction = 0 # current action
        self.inexception = -1 # whether under exception, -1: no, 0~: actionid
        self.exceptions = [] # list of exception actions in task
        self.intemp = -1 # whether under temp action, -1: no, 0~: actionid
        self.temps = [] # list of temp actions in task
        self.temptype = __tempexp__ # which temp situation
        self.intrp = False # whether under interruption planning
        self.intrpat = -1 # which action was interrupted
        self.intrpto = -1 # till where to proceed
        self.nodes = [] # boundings of action nodes, used for mouseCb

q = [TaskBox() for i in range(__maxtasks__)]
qLock = Lock()

def createTask(msg, text, active, k):
    global q
    q[k].active = active
    q[k].entid = msg.entityid
    q[k].text = text
    q[k].actions = msg.actions
    q[k].curaction = 0
    q[k].inexception = -1
    q[k].exceptions = msg.exceptions
    q[k].intemp = -1
    q[k].temps = msg.temps
    q[k].temptype = __tempexp__
    q[k].intrp = False
    q[k].intrpat = -1
    if k >= __maxtasks__: # for debug
        print 'unexpected reference %d' % k

def activateCb(msg): #VpActivate
    global q
    qLock.acquire()
    # check if activate is in queue
    for k in range(len(q)):
        if q[k].entid == msg.entityid:
            if len(q[k].actions) == 1:
                createTask(msg, 'task', 1, k)
            else:
                q[k].active = 1
                q[k].text = 'task'
            qLock.release()
            return
    # find non-active and activate
    for k in range(len(q)):
        if q[k].active < 0:
            createTask(msg, 'task', 1, k)
            break
    qLock.release()

def cleanAllCb(msg): #Empty
    global q
    qLock.acquire()
    q = None
    q = [TaskBox() for i in range(__maxtasks__)]
    qLock.release()

def addToQueueCb(msg): #Int32
    global q
    qLock.acquire()
    # find non-active and queue
    vmsg = VpActivate(entityid=msg.data, actions=['?'])
    for k in range(len(q)):
        if q[k].active < 0:
            createTask(vmsg, 'queue', 0, k)
            break
    qLock.release()

def deactivateCb(msg): #Empty
    global q
    qLock.acquire()
    for k in range(len(q)):
        if q[k].active == 1:
            q[k] = TaskBox()
            break
    qLock.release()

def updateCb(msg): #Int32
    global q
    qLock.acquire()
    for k in range(len(q)):
        if q[k].active == 1:
            if q[k].inexception >= 0:
                if q[k].intemp >= 0:
                    if q[k].temptype == __tempexp__:
                        q[k].intemp += 1
                        break
                # if __temptask__, update inexception
                q[k].inexception += 1
                if q[k].inexception >= len(q[k].exceptions):
                    q[k].inexception = -1
            elif q[k].intemp >= 0:
                q[k].intemp += 1
                if q[k].temptype == __temptask__:
                    # else, temptask state must be kept for reversing
                    if q[k].intemp >= len(q[k].temps):
                        q[k].intemp = -1
            elif q[k].temptype == __tempmove__:
                q[k].temptype = __temponmove__ # switch to on move
            elif q[k].temptype == __temponmove__:
                q[k].temptype = __tempexp__ # temp move is single node, end
            else:
                q[k].curaction = msg.data
            break
    qLock.release()

def toQueueCb(msg): #Int32
    global q
    qLock.acquire()
    # find active and send to queue
    for k in range(len(q)):
        if q[k].active == 1:
            q[k].active = 0
            q[k].entid = msg.data
            q[k].text = 'queue'
            q[k].inexception = -1 # escape exception
            # __temptask__ should keep current point when returning
            if not q[k].temptype == __temptask__:
                if q[k].intemp >= 0: # initiate for reverse temp
                    q[k].intemp = 0
            if not q[k].temptype == __tempmove__: # do not overwrite __tempmove__
                q[k].temptype = __temptask__ # other types of temp is cleared
            q[k].intrp = False # escape interruption flag
            break
    qLock.release()

def enterInterruptionCb(msg): #VpConnect
    global q
    qLock.acquire()
    for k in range(len(q)):
        if q[k].active == 1:
            q[k].intrp = True
            if msg.at >= 0: # if not replan in interrupt
                q[k].intrpat = msg.at
            q[k].intrpto = msg.to
            break
    qLock.release()

def enterExceptionCb(msg): #Empty
    global q
    qLock.acquire()
    for k in range(len(q)):
        if q[k].active == 1:
            q[k].inexception = 0
            break
    qLock.release()

def enterTempCb(msg): #String
    global q
    qLock.acquire()
    for k in range(len(q)):
        if q[k].active == 1:
            if msg.data == 'interrupted':
                q[k].intemp = 0
                q[k].temptype = __tempintrp__
            elif msg.data == 'tmpmove':
                q[k].temptype = __tempmove__
            else:
                q[k].intemp = 0
                q[k].temptype = __tempexp__
            break
    qLock.release()

subwinname_ = ''
destroysub_ = False
input_ = ''
digitsboxes_ = [Node('usedhands: ',0.0,0.0,1.0,0.2),
                Node('',0.0,0.2,0.33,0.4),Node('',0.33,0.2,0.67,0.4),Node('',0.67,0.2,1.0,0.4),
                Node('',0.0,0.4,0.33,0.6),Node('',0.33,0.4,0.67,0.6),Node('',0.67,0.4,1.0,0.6),
                Node('1',0.0,0.6,0.33,0.8),Node('2',0.33,0.6,0.67,0.8),Node('',0.67,0.6,1.0,0.8),
                Node('0',0.0,0.8,0.33,1.0),Node('ce',0.33,0.8,0.67,1.0),Node('enter',0.67,0.8,1.0,1.0)]
def subMouseCbDigits(event, x, y, flags, param):
    global input_
    global digitsboxes_
    global destroysub_
    width=480
    height=480
    if event == cv2.EVENT_LBUTTONDOWN:
        for db in digitsboxes_:
            if width*db.xs <= x and x <= width*db.xe and height*db.ys <= y and y <= height*db.ye:
                if db.actionid == 'enter':
                    if digitsboxes_[0].actionid == 'usedhands: ':
                        break # do not allow!
                    input_ += ',' + digitsboxes_[0].actionid[-1]
                    digitsboxes_[0].actionid = 'usedhands: '
                    destroysub_ = True
                elif db.actionid == 'ce':
                    digitsboxes_[0].actionid = digitsboxes_[0].actionid[:-1]
                elif len(db.actionid) == 1:
                    digitsboxes_[0].actionid = 'usedhands: ' + db.actionid
                    break
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:] = (255, 255, 255)
        for bb in digitsboxes_:
            boxth = 2
            cv2.rectangle(img, (int(width*bb.xs)+(boxth >> 1), int(height*bb.ys)+(boxth >> 1)),
                          (int(width*bb.xe)-(boxth >> 1), int(height*bb.ye)-(boxth >> 1)), __gray__, boxth)
            cv2.putText(img, bb.actionid, (int(width*bb.xs)+10, int(height*bb.ys)+30), cv2.FONT_HERSHEY_SIMPLEX, 1, __gray__, 1, LINE_AA)
        cv2.imshow(subwinname_, img)

def createDigits(n):
    global input_
    cv2.namedWindow(subwinname_, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(subwinname_, subMouseCbDigits)
    width=480
    height=480
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = (255, 255, 255)
    for bb in digitsboxes_:
        boxth = 2
        cv2.rectangle(img, (int(width*bb.xs)+(boxth >> 1), int(height*bb.ys)+(boxth >> 1)),
                      (int(width*bb.xe)-(boxth >> 1), int(height*bb.ye)-(boxth >> 1)), __gray__, boxth)
        cv2.putText(img, bb.actionid, (int(width*bb.xs)+10, int(height*bb.ys)+30), cv2.FONT_HERSHEY_SIMPLEX, 1, __gray__, 1, LINE_AA)
    cv2.imshow(subwinname_, img)
    input_ = n.actionid

def mouseCb(event, x, y, flags, param):
    global subwinname_
    if not mouse_active_:
        return
    if event == cv2.EVENT_LBUTTONDOWN:
        qLock.acquire()
        if __resize__:
            x = (x << 1)
            y = (y << 1)
        for k in range(len(q)):
            if q[k].active == 1:
                for n in q[k].nodes:
                    if n.xs <= x and x <= n.xe and n.ys <= y and y <= n.ye:
                        subwinname_ = q[k].actions[int(n.actionid)]
                        createDigits(n)
                        break
        qLock.release()

__create_digits__ = False
def uistartCb(msg):
    global subwinname_
    global __create_digits__
    global mouse_active_
    if msg.data == 'uiExceptionHandle':
        qLock.acquire()
        for k in range(len(q)):
            if q[k].active == 1:
                for n in q[k].nodes:
                    if 'tmp' in n.actionid:
                        subwinname_ = q[k].temps[int(n.actionid[3:])]
                        __create_digits__ = True
                        break
        qLock.release()
        if not __create_digits__:
            mouse_active_ = True

backcopy_ = None
targetid_ = ''
phase_ = 'non'
bridge=CvBridge()
def canvasCb(msg):
    global backcopy_
    global targetid_
    global phase_
    if 'non' in msg.header.frame_id and msg.header.frame_id[3:] != targetid_:
        return
    backcopy_ = bridge.imgmsg_to_cv2(msg, 'bgr8')
    backcopy_ = backcopy_[...,::-1] # somehow encoding rgb8 does not work
    targetid_ = msg.header.frame_id[3:]
    phase_ = msg.header.frame_id[:3]


if __name__ == '__main__':
    rospy.init_node('negomo_viewplan')
    input_task_from_viewer = rospy.get_param('~use_input', False)
    if input_task_from_viewer:
        rospy.Service('/negomo/interactivewi/defaultinteraction', PlannerDefaultInteractionCall, tinput.taskInputCb)

    rospy.Subscriber('/negomo/vp/cleanall', Empty, cleanAllCb)
    rospy.Subscriber('/negomo/vp/activate', VpActivate, activateCb)
    rospy.Subscriber('/negomo/vp/addToQueue', Int32, addToQueueCb)
    rospy.Subscriber('/negomo/vp/update', Int32, updateCb)
    rospy.Subscriber('/negomo/vp/toQueue', Int32, toQueueCb)
    rospy.Subscriber('/negomo/vp/enterInterruption', VpConnect, enterInterruptionCb)
    rospy.Subscriber('/negomo/vp/enterException', Empty, enterExceptionCb)
    rospy.Subscriber('/negomo/vp/enterTemp', String, enterTempCb)
    rospy.Subscriber('/negomo/vp/deactivate', Empty, deactivateCb)
    rospy.Subscriber('/negomo/vp/uistart', String, uistartCb)

    rospy.Subscriber('/negomo/vp/canvas', Image, canvasCb)
    rospy.Subscriber('/negomo/vp/interaction', String, wi.interactionCb)

    cv2.namedWindow('taskscreen', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('taskscreen', mouseCb)

    pub = rospy.Publisher('/negomo/vp/userinput', String, queue_size=10)

    # below for debug:

    # q[0].active = 1 
    # q[0].text = 'task' 
    # q[0].actions = ['init','move','pay','finish'] 
    # q[0].curaction = 2 
    # q[0].inexception = 1 
    # q[0].exceptions = ['gohelp','talk','solve'] 
    # q[0].intemp = 1 
    # q[0].temps = ['tmp0','tmp1','tmp2'] 
    # if q[0].active: 
    #     q[0].temptype = __tempexp__ 
    # else: 
    #     q[0].temptype = __temptask__ 

    # q[1].active = 0 
    # q[1].text = 'queue' 
    # q[1].actions = ['init','move','loop','pick','place','loop','finish'] 
    # q[1].curaction = 3 
    # q[1].inexception = -1 
    # q[1].exceptions = ['gohelp','talk','solve'] 
    # q[1].intemp = 1 
    # q[1].temps = ['tmp0','tmp1','tmp2'] 
    # q[1].temptype = __temptask__ 

    # q[2].active = 0
    # q[2].text = 'queue' 
    # q[2].actions = ['init','move','loop','pick','place','loop','finish'] 
    # q[2].curaction = 3 
    # q[2].inexception = -1 
    # q[2].exceptions = ['gohelp','talk','solve'] 
    # q[2].intemp = -1 
    # q[2].temps = ['tmp0','tmp1','tmp2'] 
    # q[2].temptype = __temptask__ 
    # q[2].intrp = True 
    # q[2].intrpat = 4  
    # q[2].intrpto = 3

    while not rospy.is_shutdown():
        # create digits window from ros callback (fails if done in callback)
        if __create_digits__:
            __create_digits__ = False
            createDigits(Node('tmp',0,0,0,0))

        # destroy sub windows if any exists
        if destroysub_:
            pub.publish(input_)
            cv2.destroyWindow(subwinname_)
            input_ = ''
            destroysub_ = False
            mouse_active_ = False

        # if task viewer input used instead of speech
        if input_task_from_viewer:
            tinput.run()

        img = np.zeros((__height__, __width__, 3), dtype=np.uint8)
        img[:] = (255, 255, 255)
        dy = __height__/(__maxtasks__+1)
        skip = 0

        # draw person estimation if any
        if not targetid_ == '':
            # draw graph
            gheight = backcopy_.shape[0]
            gwidth = backcopy_.shape[1]
            x = __width__ - gwidth - 5
            y = 0
            img[y:gheight, x:__width__-5] = backcopy_
            # draw phase
            if phase_ == 'non':
                color = (200, 200, 200)
            elif phase_ == 'pre':
                color = (0, 255, 255)
            elif phase_ == 'dur':
                color = (0, 255, 0)
            # rady = int(dy * 0.15)
            # radx = int(rady * 1.8)
            # cv2.ellipse(img, (x+radx, y+rady), (radx, rady), 0.0, 0, 360, color, 1, LINE_AA)
            cv2.putText(img, 'pers'+targetid_, (x+5, y+30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 1, LINE_AA)

        # draw interaction nodes if any
        if wi.drawnodes_:
            x = __taskwidth__ + 10
            y = 180
            wi.imglock_.acquire()
            img[y:y+wi.nodesimg_.shape[0], x:x+wi.nodesimg_.shape[1]] = wi.nodesimg_
            wi.imglock_.release()

        # draw tasks
        qLock.acquire()
        for i in range(__maxtasks__):
            # draw box
            if q[i].active < 1:
                th = 1
                boxth = 2
                color = (200, 200, 200) # gray out
                cv2.rectangle(img, ((boxth >> 1), dy*(i+skip)+(boxth >> 1)), (__taskwidth__-(boxth >> 1), dy*(i+skip+1)-(boxth >> 1)), color, boxth)
            else:
                th = 2
                boxth = 6
                color = (215, 170, 142)
                cv2.rectangle(img, ((boxth >> 1), dy*i+(boxth >> 1)), (__taskwidth__-(boxth >> 1), dy*(i+2)-(boxth >> 1)), color, boxth)
            cv2.putText(img, q[i].text, (10, dy*(i+skip)+30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, th, LINE_AA)
            if q[i].active == -1: # no further draw when not active
                continue
            # clean nodes
            q[i].nodes = []
            # draw nodes
            rady = int(dy * 0.15)
            radx = int(rady * 1.5)
            offset = 0
            # draw action nodes
            for j in range(len(q[i].actions)):
                x = 20+radx+j*(2*radx+25)
                y = dy*(i+skip)+50+rady
                th = 1
                # color settings
                if q[i].active == 0:
                    if j < q[i].curaction:
                        acolor = __done__
                    elif j == q[i].curaction:
                        if q[i].active == 0 and q[i].intemp >=0: # always __temptask__
                            offset = len(q[i].temps)
                            break
                            acolor = __done__
                    elif j == q[i].curaction + 1 and q[i].temptype == __tempmove__ or q[i].temptype == __temponmove__:
                        offset = 1
                        break
                    else:
                        acolor = __pending__
                else:
                    if j == q[i].curaction:
                        if q[i].intemp >= 0 and q[i].temptype == __temptask__:
                            offset = len(q[i].temps)
                            break
                        elif q[i].inexception >= 0:
                            acolor = __taskerror__ # in error
                        elif q[i].temptype == __tempmove__ or q[i].temptype == __temponmove__: # recovering from away
                            acolor = __taskdone__
                        else:
                            acolor = __tasknow__ # all is well
                    elif j == q[i].curaction + 1 and q[i].temptype == __tempmove__ or q[i].temptype == __temponmove__:
                        offset = 1
                        break
                    elif j < q[i].curaction:
                        acolor = __taskdone__
                    else:
                        acolor = __pending__
                    # draw intrp marks
                    if q[i].intrp:
                        if j == q[i].intrpat: # had interruption
                            if q[i].intrpat == q[i].intrpto:
                                cv2.circle(img, (x+(rady >> 2)+2, y-rady-(rady>>2)-10), (rady >> 2), __taskintrp__, -1, LINE_AA)
                            else:
                                cv2.circle(img, (x, y-rady-(rady>>2)-10), (rady >> 2), __taskintrp__, -1, LINE_AA)
                        if j == q[i].intrpto: # interruption proceed goal
                            if q[i].intrpat == q[i].intrpto:
                                cv2.circle(img, (x-(rady >> 2)-2, y-rady-(rady>>2)-10), (rady >> 2), __taskto__, -1, LINE_AA)
                            else:
                                cv2.circle(img, (x, y-rady-(rady>>2)-10), (rady >> 2), __taskto__, -1, LINE_AA)
                # draw main actions
                cv2.ellipse(img, (x, y), (radx, rady), 0.0, 0, 360, acolor, th, LINE_AA)
                q[i].nodes += [Node(str(j), x-radx, y-rady, x+radx, y+rady)]
                cv2.putText(img, q[i].actions[j], (x-radx+5, y-rady+30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, acolor, th, LINE_AA)
                if j < len(q[i].actions) - 1:
                    if j == q[i].curaction:
                        cv2.line(img, (x+radx,y), (x+radx+25,y), __pending__, 2)
                    else:
                        cv2.line(img, (x+radx,y), (x+radx+25,y), acolor, 2)
            # draw temp nodes if any
            if q[i].temptype == __temptask__ and q[i].intemp >=0:
                # draw temp actions
                for j in range(len(q[i].temps)):
                    if q[i].active == 0:
                        acolor = __pending__
                    elif j < q[i].intemp:
                        acolor = __taskdone__
                    elif j == q[i].intemp:
                        if q[i].inexception >= 0:
                            acolor = __taskerror__
                        else:
                            acolor = __tasknow__
                    else:
                        acolor = __pending__
                    # draw temp actions
                    x = 20+radx+(q[i].curaction+j)*(2*radx+25)
                    y = dy*(i+skip)+50+rady
                    cv2.ellipse(img, (x, y), (radx, rady), 0.0, 0, 360, acolor, th, LINE_AA)
                    q[i].nodes += [Node('tmp'+str(j), x-radx, y-rady, x+radx, y+rady)]
                    cv2.putText(img, q[i].temps[j], (x-radx+5, y-rady+30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, acolor, th, LINE_AA)
                    cv2.line(img, (x+radx,y), (x+radx+25,y), acolor, 2)
                # draw remaing main actions
                for j in range(q[i].curaction, len(q[i].actions)):
                    # color settings
                    acolor = __pending__
                    # draw main actions
                    x = 20+radx+(offset+j)*(2*radx+25)
                    y = dy*(i+skip)+50+rady
                    cv2.ellipse(img, (x, y), (radx, rady), 0.0, 0, 360, acolor, th, LINE_AA)
                    q[i].nodes += [Node(str(j), x-radx, y-rady, x+radx, y+rady)]
                    cv2.putText(img, q[i].actions[j], (x-radx+5, y-rady+30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, acolor, th, LINE_AA)
                    if j < len(q[i].actions) - 1:
                        if j == q[i].curaction:
                            cv2.line(img, (x+radx,y), (x+radx+25,y), __pending__, 2)
                        else:
                            cv2.line(img, (x+radx,y), (x+radx+25,y), acolor, 2)
            elif q[i].temptype == __tempmove__ or q[i].temptype == __temponmove__:
                # draw temp action
                if q[i].active == 0:
                    acolor = __pending__
                else:
                    acolor = __tasknow__
                x = 20+radx+(q[i].curaction+1)*(2*radx+25)
                y = dy*(i+skip)+50+rady
                cv2.ellipse(img, (x, y), (radx, rady), 0.0, 0, 360, acolor, th, LINE_AA)
                q[i].nodes += [Node('tmp0', x-radx, y-rady, x+radx, y+rady)]
                cv2.putText(img, 'move', (x-radx+5, y-rady+30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, acolor, th, LINE_AA)
                cv2.line(img, (x+radx,y), (x+radx+25,y), acolor, 2)
                # draw remaing main actions
                for j in range(q[i].curaction+1, len(q[i].actions)):
                    # color settings
                    acolor = __pending__
                    # draw main actions
                    x = 20+radx+(offset+j)*(2*radx+25)
                    y = dy*(i+skip)+50+rady
                    cv2.ellipse(img, (x, y), (radx, rady), 0.0, 0, 360, acolor, th, LINE_AA)
                    q[i].nodes += [Node(str(j), x-radx, y-rady, x+radx, y+rady)]
                    cv2.putText(img, q[i].actions[j], (x-radx+5, y-rady+30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, acolor, th, LINE_AA)
                    if j < len(q[i].actions) - 1:
                        if j == q[i].curaction:
                            cv2.line(img, (x+radx,y), (x+radx+25,y), __pending__, 2)
                        else:
                            cv2.line(img, (x+radx,y), (x+radx+25,y), acolor, 2)
            # draw exception nodes (only for task nodes)
            if q[i].active and q[i].inexception >= 0:
                if q[i].active and q[i].intemp >= 0 and q[i].temptype == __tempexp__: # entered temp actions during exception
                    offset = q[i].curaction
                    drawnodes = q[i].inexception + 1
                elif q[i].active and q[i].intemp >= 0 and q[i].temptype == __temptask__: # entered exception in temp
                    offset = q[i].curaction + q[i].intemp
                    drawnodes = len(q[i].exceptions)
                else:
                    offset = q[i].curaction
                    drawnodes = len(q[i].exceptions)
                x = 20+radx+offset*(2*radx+25)
                y = dy*i+50+2*rady
                cv2.line(img, (x, y), (x, y+25), __done__, 2)
                for j in range(drawnodes):
                    # color settings
                    if j < q[i].inexception:
                        acolor = (204, 83, 56)
                    elif j == q[i].inexception:
                        if q[i].intemp >= 0:
                            acolor = __taskintrp__
                        else:
                            acolor = __tasknow__
                    else:
                        acolor = __pending__
                    # draw exception actions
                    x = 20+radx+(offset+j)*(2*radx+25)
                    y = dy*i+50+3*rady+25
                    cv2.ellipse(img, (x, y), (radx, rady), 0.0, 0, 360, acolor, th, LINE_AA)
                    cv2.putText(img, q[i].exceptions[j], (x-radx+5, y-rady+30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, acolor, th, LINE_AA)
                    if j < len(q[i].exceptions) - 1:
                        if j == q[i].inexception:
                            if q[i].active and q[i].intemp >= 0:
                                cv2.line(img, (x+radx,y), (x+radx+25,y), acolor, 2)
                            else:
                                cv2.line(img, (x+radx,y), (x+radx+25,y), __pending__, 2)
                        else:
                            cv2.line(img, (x+radx,y), (x+radx+25,y), acolor, 2)
                # draw temp nodes
                if q[i].active and q[i].temptype == __tempexp__ and q[i].intemp >=0:
                    for j in range(len(q[i].temps)):
                        # color settings
                        if j <= q[i].intemp:
                            acolor = __taskdone__
                        elif j == q[i].intemp:
                            acolor = __tasknow__
                        else:
                            acolor = __pending__
                        # draw temp actions
                        x = 20+radx+(q[i].curaction+q[i].inexception+1+j)*(2*radx+25)
                        y = dy*i+50+3*rady+25
                        cv2.ellipse(img, (x, y), (radx, rady), 0.0, 0, 360, acolor, th, LINE_AA)
                        cv2.putText(img, q[i].temps[j], (x-radx+5, y-rady+30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, acolor, th, LINE_AA)
                        if j < len(q[i].temps) - 1:
                            if j >= q[i].intemp:
                                cv2.line(img, (x+radx,y), (x+radx+25,y), __pending__, 2)
                            else:
                                cv2.line(img, (x+radx,y), (x+radx+25,y), acolor, 2)
            if q[i].active == 1:
                skip = 1

        qLock.release()
        if __resize__:
            img = cv2.resize(img, ((__width__ >> 1), (__height__ >> 1)))
        cv2.imshow('taskscreen', img)
        cv2.waitKey(10)
