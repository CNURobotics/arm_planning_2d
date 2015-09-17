#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
    Simple 2-link robot arm simulation for generating graphics for CS 471/510

    The MIT License (MIT)

    Copyright (c) 2015 David Conner (david.conner@cnu.edu)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

'''

import os.path
import cv2
import numpy as np
from copy import deepcopy
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt
import matplotlib.patches as patches

import sys


# Import our simple simulation
from RobotWorld import *


# This is the main body of the program

# Define links for our simple robot
#             ID   Length, color, width, parent(=None default)
link1 = Link(" 1 ", 10.0,'b',0.5)
link2 = Link(" 2 ", 6.0,'g',0.3, link1)

# Define a robot with tuple of links (2 in this case but could add more)
robot = RobotArm((link1, link2))
robot.updateLinks((np.pi/4.0, -np.pi/6.0)) # Define the initial angles

# Define obstacles
obj1 = Obstacle("A", ( -5.0,  0.0), 0.5 , [0.5,0.0,0.0])
obj2 = Obstacle("B", (  5.0,  0.0), 0.5 , [1.0,0.0,0.0])
obj3 = Obstacle("C", (  0.0, 12.0), 1.5 , [0.5,0.5,0.5])
obj4 = Obstacle("D", ( 10.0,  5.0), 0.85, [1.0,0.5,0.5])
obj5 = Obstacle("E", ( -5.0, 10.0), 0.75, [0.5,1.0,0.5])
obj6 = Obstacle("F", (-10.0,  5.0), 1.5 , [0.5,0.5,1.0])

# Store a tuple of all obstacles
obstacles = (obj1, obj2, obj3, obj4, obj5, obj6)



world = RobotWorld(robot, obstacles)

#link1.updateTip(np.pi/4.0)
#link2.updateTip(np.pi/6.0)


# Define maximum range for our workspace plots
max_len = (link1.length + link2.length)*1.5;


# Define a hand specified list of points that work for this simple environment
pts = [(0.286, -1.04), (0.5, -2.0), (1.0, -2.125), (1.5, -2.25),
       (1.48, -1.82),  (1.42, -1.42), (1.0, -0.75), (0.75, 1.0), (0.6, 1.5),
       (0.671, 2.18), (1.5, 2.18), (1.7, 1.8), (1.87, 1.6),
       (2.012, 1.369), (2.27, 0.0), (2.48, -1.26)]

pt0 = np.asarray(deepcopy(pts[0]));# convert tuple to array so we can do math
pt1 = np.asarray(deepcopy(pts[-1]));# convert tuple to array so we can do math

if (np.array_equal(pt0,pt1)):
    print "Points are equal"
else:
    print "Points are NOT equal"


# store angles and color of collision
collisions_theta1=[]
collisions_theta2=[]
collisions_colors=[]

# (x,y) positions of end effector in the workspace
workspace_x=[]
workspace_y=[]

# Positions reachable in free C-space
free_workspace_x=[]
free_workspace_y=[]

torus=None

# Convert angles to a torus surface in 3D space
def theta2xyz(t1,t2):
    return ( (6.*np.cos(t2)*np.cos(t1) + 10.0*np.cos(t1)),
             (6.*np.cos(t2)*np.sin(t1) + 10.0*np.sin(t1)),
             (6.*np.sin(t2)))

if (True): # build C-space map

    print "Define the torus ..."
    pi_diff = np.pi/100.0;#72.0;#36.0
    thetas = np.arange(-np.pi, np.pi+pi_diff, pi_diff)
    t1grid,t2grid = np.meshgrid(thetas,thetas)

    torus = theta2xyz(t1grid, t2grid)


    print "Now building the C-space map ..."
    for theta1 in thetas:
        #print "theta1=",theta1
        for theta2 in thetas:

            # update the position of the links in the world
            world.updateRobotArm((theta1,theta2));

            # Store the location of the end effector
            tip = world.robot.getEndEffector();

            workspace_x.append(tip[0])
            workspace_y.append(tip[1])

            color = world.checkCollisions()
            if (color is not None):
                collisions_theta1.append(theta1)
                collisions_theta2.append(theta2)
                collisions_colors.append(color)
            else:
                if (theta1 >= 0.0): # just plot the free space on upper half of workspace
                    free_workspace_x.append(robot.links[-1].tip[0])
                    free_workspace_y.append(robot.links[-1].tip[1])


    print "plot the C-space ..."
    fig1=plt.figure();
    ax1=fig1.gca();
    ax1.grid(True);
    ax1.set_xlim([-np.pi, np.pi])
    ax1.set_ylim([-np.pi, np.pi])
    ax1.set_aspect('equal', 'box');

    print "Now plotting ", len(collisions_theta1), " collision points for C-space obstacles ..."
    ax1.scatter(collisions_theta1,collisions_theta2,c=collisions_colors,alpha=0.5,edgecolors='none')


    if (False):
      print "Show path on C-space ..."
      Xp=[];
      Yp=[];
      for pt in pts:
        Xp.append(pt[0])
        Yp.append(pt[1])

      ax1.plot(Xp,Yp);

      ax1.plot(pts[0][0],pts[0][1],color='g',marker='o');
      ax1.plot(pts[-1][0],pts[-1][1],color='r',marker='x');

    fig1.savefig( "config_space.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

    if (True):
        # Draw black (obstacle) and white image for planning
        fig1=plt.figure();
        ax1=fig1.gca();
        ax1.grid(False);
        ax1.get_xaxis().set_visible(False)
        ax1.get_yaxis().set_visible(False)
        ax1.set_xlim([-np.pi, np.pi])
        ax1.set_ylim([-np.pi, np.pi])
        ax1.set_aspect('equal', 'box');
        ax1.axis('off')
        print "Now plotting ", len(collisions_theta1), " collision points for C-space obstacles ..."
        ax1.scatter(collisions_theta1,collisions_theta2,c='black',alpha=0.5,edgecolors='none')
        fig1.savefig( "config_space_bw.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )


if (True):
    print "Draw start ..."
    robot.updateLinks(pts[0])
    fig2=plt.figure();
    fig2.suptitle("Start")
    ax2 = fig2.gca();
    ax2.grid(True);
    ax2.set_xlim([-max_len, max_len])
    ax2.set_ylim([-max_len, max_len])
    ax2.set_aspect('equal', 'box');

    world.drawWorld(ax2) # Draw robot and obstacles
    fig2.savefig( "start_position.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )


if (True):
    print "Draw finish ..."
    robot.updateLinks(pts[-1]) # get last point in path list

    fig3=plt.figure();
    fig3.suptitle("Finish")
    ax3 = fig3.gca();
    ax3.grid(True);
    ax3.set_xlim([-max_len, max_len])
    ax3.set_ylim([-max_len, max_len])
    ax3.set_aspect('equal', 'box');

    world.drawWorld(ax3);

    fig3.savefig( "finish_position.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )


if (True):
    print "Draw intermediates ..."
    fig4=plt.figure();
    fig4.suptitle("Motion")
    ax4 = fig4.gca();
    ax4.grid(True);
    ax4.set_xlim([-max_len, max_len])
    ax4.set_ylim([-max_len, max_len])
    ax4.set_aspect('equal', 'box');

    for pt in pts:
        world.updateRobotArm(pt);
        robot.drawArm(ax4)

    for obj in obstacles:
       obj.drawObstacle(ax4)
    fig4.savefig( "motion_snapshots.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )


if (True):
    if (len(workspace_x) > 0): # C-space was built
        print "Now show the workspace with ",len(workspace_x), " points, and ",len(free_workspace_x)," free points"
        fig5=plt.figure();
        fig5.suptitle("Workspace")
        ax5 = fig5.gca();
        ax5.grid(True);
        ax5.set_xlim([-max_len, max_len])
        ax5.set_ylim([-max_len, max_len])
        ax5.set_aspect('equal', 'box');

        ax5.scatter(workspace_x,workspace_y,color='b', edgecolors='none')

        ax5.scatter(free_workspace_x,free_workspace_y,color='g',edgecolors='none')

        for obj in obstacles:
          obj.drawObstacle(ax5)
        fig5.savefig( "workspace.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

    else:
        print "No C-space map was built!"

if (torus is not None):
    print "Draw the torus ..."
    fig7=plt.figure()
    ax7 = fig7.add_subplot(111,projection='3d')
    ax7.plot_surface(torus[0],torus[1],torus[2])
    ax7.set_xlim([-max_len, max_len])
    ax7.set_ylim([-max_len, max_len])
    ax7.set_zlim([-max_len, max_len])
    ax7.set_aspect('equal', 'box');
    fig7.savefig( "torus.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

    if (len(collisions_theta1) > 0):
        print "Draw the torus with collisions ..."
        fig8=plt.figure()
        ax8 = fig8.add_subplot(111,projection='3d')
        ax8.plot_wireframe(torus[0],torus[1],torus[2],alpha=0.5,rstride=4,cstride=4,color='black')
        ax8.set_xlim([-max_len, max_len])
        ax8.set_ylim([-max_len, max_len])
        ax8.set_zlim([-max_len, max_len])
        ax8.set_aspect('equal', 'box');

        torus_objs = theta2xyz(collisions_theta1, collisions_theta2)
        ax8.scatter(torus_objs[0], torus_objs[1], torus_objs[2],
                    c=collisions_colors,alpha=0.5,edgecolors='none')

        fig8.savefig( "torus_obstacles.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

print "Close plots to continue ..."
plt.show()


#print "Done! - comment out the exit below to create images for motion video"
#sys.exit(0)

print "Make video of motion ..."

fig6=plt.figure();
fig6.suptitle("Motion")
ax6 = fig6.gca();
ax6.grid(True);
ax6.set_xlim([-max_len, max_len])
ax6.set_ylim([-max_len, max_len])
ax6.set_aspect('equal', 'box');

pt0 = np.asarray(deepcopy(pts[0]) )
robot.updateLinks(pt0);
robot.drawArm(ax6)

for obj in obstacles:
   obj.drawObstacle(ax6)

cnt=0;

fig6.savefig( "roboplan_animation_0.png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

pt0 = np.asarray(deepcopy(pts[0]));# convert tuple to array so we can do math

for pt in pts:
    pt1 = np.asarray(deepcopy(pt)) # convert tuple to array so we can do math
    print pt1
    if (not np.array_equal(pt1,pt0)):
      for s in np.arange(0.025,1.0,0.025): # interpolate along line from prior point to current
        pti = pt0 + (pt1-pt0)*s
        #print pti

        plt.cla()
        ax6.grid(True);
        ax6.set_xlim([-max_len, max_len])
        ax6.set_ylim([-max_len, max_len])
        ax6.set_aspect('equal', 'box');

        world.updateRobotArm(pti);
        world.drawWorld(ax6)

        cnt = cnt +1

        fig6.savefig( "roboplan_animation_"+str(cnt)+".png", format = "png", bbox_inches = 'tight', pad_inches = 0 )

    pt0 = pt1

print "Done!"
print ""
print "In Ubuntu, we can use the following to create an avi video from images:"
print ""
print "avconv -r 30 -i roboplan_animation_%d.png -b:v 1000k roboplan.avi"
print ""
print "Done!"
