#!/usr/bin/env python

import rospy
from negomo_enshu.msg import NegomoSensors
import pygame
import sys

if __name__ == '__main__':
    rospy.init_node('simulated_sensors')
    face_status_publisher = rospy.Publisher('/negomo/sensor/face', NegomoSensors, queue_size=10)

    pygame.init()
    screen = pygame.display.set_mode((100, 100))

    obs = 'noperson'
    while not rospy.is_shutdown():
        pressed = pygame.key.get_pressed()
        for event in pygame.event.get():
            # determin if X was clicked, or Ctrl+W or Alt+F4 was used
            if event.type == pygame.QUIT:
                sys.exit(0)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_l:
                    obs = 'looktoward'
                elif event.key == pygame.K_s:
                    obs = 'lookaway'
                elif event.key == pygame.K_e:
                    obs = 'noperson'
                elif event.key == pygame.K_t: # toggle for debug
                    obs = 'looktoward'
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_l:
                    obs = 'lookaway'
        face_status_publisher.publish([obs])
        rospy.sleep(1.0)
