#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from std_msgs.msg import Header, String
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray
from roboenvcv.msg import PersonCoordinate
import pygame
import xml.etree.ElementTree
import time

__keywidth__ = 800
__keyheight__ = 500

__keyboard__ = 'keyboard'
__color__ = (200, 200, 200)

keyboxes_ = ''

createkeyboard_ = False
destroykeyboard_ = False

img_ = None
keyboxes_ = []

class Key:
    def __init__(self, _key, _xs, _ys, _xe, _ye):
        self.key = _key
        self.xs = _xs
        self.ys = _ys
        self.xe = _xe
        self.ye = _ye

def keyMouseCb(event, x, y, flags, param):
    global destroykeyboard_
    if event == cv2.EVENT_LBUTTONDOWN:
        for kb in keyboxes_:
            if __keywidth__*kb.xs <= x and x <= __keywidth__*kb.xe and __keyheight__*kb.ys <= y and y <= __keyheight__*kb.ye:
                pub_detected_speech.publish(kb.key)
                destroykeyboard_ = True
                break

def createKeyboard():
    img = np.zeros((__keyheight__, __keywidth__, 3), dtype=np.uint8)
    img[:] = (255, 255, 255)
    for bb in keyboxes_:
        boxth = 2
        cv2.rectangle(img, (int(__keywidth__*bb.xs)+(boxth >> 1), int(__keyheight__*bb.ys)+(boxth >> 1)), (int(__keywidth__*bb.xe)-(boxth >> 1), int(__keyheight__*bb.ye)-(boxth >> 1)), __color__, boxth)
        cv2.putText(img, bb.key, (int(__keywidth__*bb.xs)+10, int(__keyheight__*bb.ys)+30), cv2.FONT_HERSHEY_SIMPLEX, 1, __color__, 1, cv2.LINE_AA)
    cv2.imshow(__keyboard__, img)
    cv2.namedWindow(__keyboard__, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(__keyboard__, keyMouseCb)

def getSpeechChoices(msg):
    global keyboxes_
    global createkeyboard_
    try:
        # create keyboard
        tfile = templatedir + msg.data.split('\\')[-1]
        e = xml.etree.ElementTree.parse(tfile).getroot()
        div = 1.0 / len(e[0][0])
        y = 0.0
        keyboxes_ = []
        for item in e[0][0]:
            for child in item:
                if 'tag' in child.tag:
                    # remove " in tag -> [1:-1]
                    keyboxes_ += [Key(child.text[1:-1],0.0,y,1.0,y+div)]
                    y += div
                    break
        print keyboxes_
        while destroykeyboard_: # wait for window to completely shutdown
            time.sleep(0.1)
        createkeyboard_ = True
    except:
        print 'warning: bad template file'

def run():
    global vx_
    global vy_
    global target_
    global poses_
    global createkeyboard_
    global destroykeyboard_
    global exist_
    rx = 0
    ry = 0
    pressed = pygame.key.get_pressed()
    for event in pygame.event.get():
        # determin if X was clicked, or Ctrl+W or Alt+F4 was used
        if event.type == pygame.QUIT:
            sys.exit(0)
        elif event.type == pygame.KEYDOWN:
            # switch human control
            if event.key == pygame.K_f:
                target_ += 1
                if target_ == len(exist_):
                    target_ = 0
            elif event.key == pygame.K_j:
                target_ -= 1
                if target_ == -1:
                    target_ = len(exist_) - 1

            # look control
            if event.key == pygame.K_e:
                exist_[target_] = not exist_[target_]

            # face direction control
            if event.key == pygame.K_w:
                vy_ = 0.05
                ry += 1
                pass
            if event.key == pygame.K_a:
                vx_ = -0.05
                rx -= 1
                pass
            if event.key == pygame.K_s:
                vy_ = -0.05
                ry -= 1
                pass
            if event.key == pygame.K_d:
                vx_ = 0.05
                rx += 1
                pass
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_w:
                vy_ = 0.0
                pass
            if event.key == pygame.K_a:
                vx_ = 0.0
                pass
            if event.key == pygame.K_s:
                vy_ = 0.0
                pass
            if event.key == pygame.K_d:
                vx_ = 0.0
                pass

    # speech menu control
    if createkeyboard_:
        createKeyboard()
        createkeyboard_ = False
    if destroykeyboard_:
        cv2.destroyWindow(__keyboard__)
        destroykeyboard_ = False

    # update
    poses_[target_].position.x += vx_
    poses_[target_].position.y += vy_
    if rx != 0 or ry != 0:
        if rx > 0:
            if ry == 0:
                rx = 1
                ry = 0
            else:
                if ry > 0: # w + d
                    rx = 0.923879
                    ry = 0.382683
                else: # s + d
                    rx = -0.923879
                    ry = 0.382683
        elif rx == 0:
            rx = 0.707107
            if ry > 0:
                ry = 0.707107
            else:
                ry = -0.707107
        else:
            if ry == 0:
                rx = 0
                ry = 1
            else:
                if ry > 0: # w + a
                    rx = 0.382683
                    ry = 0.923879
                else: # s + a
                    rx = 0.382683
                    ry = -0.923879
        poses_[target_].orientation = Quaternion(rx, 0, 0, ry)
    for i, pose in enumerate(poses_):
        if exist_[i]:
            pub_detected_person.publish(header=Header(stamp=rospy.get_rostime(), frame_id='map'),
                                        id=str(i),
                                        position3d_map=pose.position,
                                        position3d_camera=Point(0, 0, 0),
                                        pose3d_camera=Quaternion(1, 0, 0, 0),
                                        map_to_camera=pose)

if __name__ == '__main__':
    rospy.init_node('simulated_person')
    num = rospy.get_param('~num_people', 1)
    start_positions = rospy.get_param('~start', '').split(';')
    templatedir = rospy.get_param('~templatedir', '')
    rospy.Subscriber('/settings/speech', String, getSpeechChoices)

    print templatedir

    pub_detected_person = rospy.Publisher('/roboenvcv/personcoordinate/global/withid', PersonCoordinate, queue_size=10)
    pub_detected_speech = rospy.Publisher('/detected/speech/template', String, queue_size=10)

    pygame.init()
    screen = pygame.display.set_mode((100, 100))
    target_ = 0
    exist_ = [True for x in range(num)]
    poses_ = [Pose() for x in range(num)]
    vx_ = 0.0
    vy_ = 0.0

    for i, s in enumerate(start_positions):
        if i >= len(poses_):
            break
        p = s.split(',')
        if len(p) != 7:
            continue
        poses_[i] = Pose(position=Point(float(p[0]),float(p[1]),float(p[2])),
                         orientation=Quaternion(float(p[6]),float(p[3]),float(p[4]),float(p[5])))

    while not rospy.is_shutdown():
        run()
        cv2.waitKey(1)
        rospy.sleep(0.1)
