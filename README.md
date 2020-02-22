# arm_planning_2d
Simple python scripts for generating images of 2D robot arm among circular obstacles

Run ./roboplan.py to load generate the configuration space obstacles and simulate a simple specified path.

The code is intended to illustrate the topology and C-space obstacles for simple environment, and to serve as a foundation for planning assignments.

In this current version, the link sizes and obstacle locations and sizes are hard coded in the main function, but they are easy to modify.

The code has been tested on Ubuntu 14.04 and MAC OSX with Python 2.7, OpenCV (cv2), Numpy, and Matplotlib.
