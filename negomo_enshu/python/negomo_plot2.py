#!/usr/bin/env python

import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import numpy as np
import rospy
from negomo_enshu.msg import *
from std_msgs.msg import *
from threading import Lock
from datetime import datetime
from sensor_msgs.msg import Image


def savePlot(msg):
    if msg.data > len(fig):
        print 'tried to save bad fig'
        return
    date = datetime.now()
    timestring = str(date.year) + '-' + str(date.month) + '-' + str(date.day) + \
                 '-' + str(date.hour) + '-' + str(date.minute) + '-' + str(date.second)
    savefile = 'fig' + str(msg.data) + timestring + '.pdf'
    figlock.acquire()
    fig[msg.data].savefig(savefile, bbox_inches='tight')
    figlock.release()
    print 'save to ' + savefile

def drawPlot(msg):
    global fig
    global ax0
    global phase

    idx = msg.target

    # timeline alignment
    timeline = []
    intents = []
    colors = []
    tlridx = 0
    for i, t in enumerate(msg.timelinep):
        while tlridx < len(msg.timeliner) and msg.timeliner[tlridx] < t:
            timeline += [msg.timeliner[tlridx]]
            colors += ['b']
            intents += [-msg.intentr[tlridx]]
            tlridx += 1
        if tlridx < len(msg.timeliner) and t == msg.timeliner[tlridx]:
            tlridx += 1
            colors += ['0.5']
        elif msg.intentp[i] < 0:
            colors += ['0.5']
        else:
            colors += ['r']
        intents += [msg.intentp[i]]
        timeline += [t]
    while tlridx < len(msg.timeliner):
        timeline += [msg.timeliner[tlridx]]
        if msg.intentr[tlridx] > 0:
            colors += ['0.5']
        else:
            colors += ['b']
        intents += [-msg.intentr[tlridx]]
        tlridx += 1

    # plot
    cbef = colors[0]
    sidx = 0
    figlock.acquire()
    # record phase from msg.color (used in nocanvas mode only)
    if msg.color == 'b':
        phase[idx] = 'non'
    elif msg.color == 'y':
        phase[idx] = 'pre'
    elif msg.color == 'g':
        phase[idx] = 'dur'
    # record plot
    ax0[idx].cla()
    ax0[idx].set_xlim([-20.0, 0.0])
    ax0[idx].set_ylim([-1.1, 1.1])
    for i, color in enumerate(colors):
        if color != cbef:
            ax0[idx].plot(timeline[sidx:i], intents[sidx:i], cbef)
            cbef = color
            sidx = i-1
    ax0[idx].plot(timeline[sidx:len(colors)], intents[sidx:len(colors)], cbef)
    figlock.release()

def drawSeq(msg):
    global timeline
    global obsarray
    global ax1
    global actarray
    global ax2

    idx = int(msg.data.split(';')[0])
    tnow = float(msg.data.split(';')[-1].split(':')[0])
    dat = msg.data.split(';')[-1].split(':')[-1].split(',')[-1]
    obs = dat.split('_')[0]
    act = dat.split('_')[1]

    # check for time jump back (occurs when replaying bag)
    if len(timeline[idx]) > 0 and tnow < timeline[idx][-1]:
        timeline[idx] = []
        obsarray[idx] = []
        actarray[idx] =[]

    # remove timeouts
    keeptrack = 0
    for i, t in enumerate(timeline[idx]):
        if tnow - t > 40.0:
            continue
        else:
            keeptrack = i
            break
    timeline[idx] = timeline[idx][keeptrack:]
    obsarray[idx] = obsarray[idx][keeptrack:]
    actarray[idx] = actarray[idx][keeptrack:]

    figlock.acquire()

    # append time
    timeline[idx].append(tnow)
    # append obs (hard-coded)
    if obs == 'looktoward':
        obsarray[idx].append('b')
    elif obs == 'lookaway':
        obsarray[idx].append('0.5')
    else:
        obsarray[idx].append('w')
    # convert obs array to segments
    ax1[idx].cla()
    obef = obsarray[idx][0]
    tend = timeline[idx][-1]
    tstart = timeline[idx][0] - tend
    tsum=0
    if tstart > -20:
        tsum = tstart + 20
        ax1[idx].barh([0], [tsum], color='w', left=0)
    elif tstart < -20:
        tstart = -20
    for i, o in enumerate(obsarray[idx]):
        if o != obef:
            tcur = timeline[idx][i-1] - tend
            if tcur >= -20:
                ax1[idx].barh([0], [tcur - tstart], color=obef, left=tsum)
                tsum += tcur - tstart
                tstart = tcur
            obef = o
    ax1[idx].barh([0], [-tstart], color=obef, left=tstart+20)
    ax1[idx].set_yticklabels(['observation'])
    ax1[idx].get_xaxis().set_visible(False)

    # append act (hard-coded)
    if 'proactive' in act:
        if int(act[-1]) > 0:
            actarray[idx].append((0, 0, 1, 1))
        else:
            actarray[idx].append((0, 0, 1, 0.1))
    elif 'reactive' in act:
        if int(act[-1]) > 0:
            actarray[idx].append((1, 0, 0, 1))
        else:
            actarray[idx].append((1, 0, 0, 0.1))
    else:
        actarray[idx].append('0.5')
    # convert act array to segments
    ax2[idx].cla()
    abef = actarray[idx][0]
    tstart = timeline[idx][0] - tend
    tsum = 0
    if tstart > -20:
        tsum = tstart + 20
        ax2[idx].barh([0], [tsum], color='w', left=0)
    elif tstart < -20:
        tstart = -20
    for i, a in enumerate(actarray[idx]):
        if a != abef:
            tcur = timeline[idx][i-1] - tend
            if tcur >= -20:
                ax2[idx].barh([0], [tcur - tstart], color=abef, left=tsum)
                tsum += tcur - tstart
                tstart = tcur
            abef = a
    ax2[idx].barh([0], [-tstart], color=abef, left=tstart+20)
    ax2[idx].set_yticklabels(['robot behavior'])
    ax2[idx].get_xaxis().set_visible(False)

    figlock.release()

def erasePlot(msg):
    for i, status in enumerate(msg.data):
        if i >= num:
            break
        if status == 'noperson':
            figlock.acquire()
            ax0[i].cla()
            ax0[i].set_xlim([-20.0, 0.0])
            ax0[i].set_ylim([-1.1, 1.1])
            ax0[i].plot([-20.0, 0.0], [-1.0, 1.0], '0.5')
            ax1[i].cla()
            ax1[i].barh([0], [1.0], color='0.5')
            ax1[i].set_yticklabels(['observation'])
            ax1[i].get_xaxis().set_visible(False)
            ax2[i].cla()
            ax2[i].barh([0], [1.0], color='0.5')
            ax2[i].set_yticklabels(['robot behavior'])
            ax2[i].get_xaxis().set_visible(False)
            figlock.release()

if __name__ == '__main__':
    rospy.init_node('negomo_plot')
    num = rospy.get_param('~max_targets')
    nocanvas = rospy.get_param('~no_canvas', False)

    timeline = [[] for x in range(0, num)]
    obsarray = [[] for x in range(0, num)]
    actarray = [[] for x in range(0, num)]

    fig = [None for x in range(0, num)]
    ax0 = [None for x in range(0, num)]
    ax1 = [None for x in range(0, num)]
    ax2 = [None for x in range(0, num)]
    phase = ['non' for x in range(0, num)] # for nocanvas mode

    if not nocanvas: # if nocanvas mode, don't plot
        plt.ion()
    gs = gridspec.GridSpec(3, 1, height_ratios=[5, 1, 1], left=0.25, right=0.95, hspace=0.5)

    for i in range(0, len(fig)):
        fig[i] = plt.figure(figsize=(6, 2))
        ax0[i] = fig[i].add_subplot(gs[0])
        ax1[i] = fig[i].add_subplot(gs[1])
        ax2[i] = fig[i].add_subplot(gs[2])
        ax0[i].plot([-20.0, 0.0], [-1.0, 1.0], '0.5')
        ax1[i].barh([0], [1.0], color='0.5')
        ax1[i].set_yticklabels(['observation'])
        ax1[i].get_xaxis().set_visible(False)
        ax2[i].barh([0], [1.0], color='0.5')
        ax2[i].set_yticklabels(['robot behavior'])
        ax2[i].get_xaxis().set_visible(False)

    rospy.Subscriber('/negomo/plot', NegomoPlot, drawPlot, queue_size=1)
    rospy.Subscriber('/negomo/observation_sequence', String, drawSeq, queue_size=1)
    rospy.Subscriber('/negomo/plot/save', Int32, savePlot, queue_size=1)
    rospy.Subscriber('/negomo/sensor/face', NegomoSensors, erasePlot, queue_size=1)

    if nocanvas:
        canvas = [None for i in range(len(fig))]
        for i in range(len(fig)):
            canvas[i] = FigureCanvas(fig[i])
        pub = rospy.Publisher('/negomo/vp/canvas', Image, queue_size=5)
        r = rospy.Rate(10) # don't publish too fast

    figlock = Lock()

    while not rospy.is_shutdown():
        figlock.acquire()
        if nocanvas:
            for i in range(0, len(fig)):
                canvas[i].draw()
                image = np.fromstring(canvas[i].tostring_rgb(), dtype='uint8')
                width, height = fig[i].get_size_inches() * fig[i].get_dpi()
                pub.publish(Image(header=Header(frame_id=phase[i]+str(i)),
                                  height=height, width=width,
                                  encoding='bgr8', is_bigendian=False,
                                  step=3*width,
                                  data=image.tolist()))
            r.sleep()
        else:
            for i in range(0, len(fig)):
                fig[i].canvas.draw()
        figlock.release()
