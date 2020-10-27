Python 3.6.9 (default, Oct  8 2020, 12:12:24) 
[GCC 8.4.0] on linux
Type "help", "copyright", "credits" or "license()" for more information.
>>> #!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
from qualisys.msg import Subject

def plot_x(msg):
    global counter
    if counter % 10 == 0:
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(msg.position.y, msg.position.x, '*')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter")
    rospy.Subscriber("position_measurements", Subject, plot_x)
    plt.ion()
    plt.show()
    rospy.spin()
