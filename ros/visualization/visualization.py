#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from math import *
from inverted_pendulum.msg import pendulum


def callback(data):
    x_position = data.x_position
    theta = data.theta
    plt.cla()
    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
    plt.plot(x_position, 0, 'bo', markersize=3)
    plt.plot([x_position, x_position + 0.5*sin(theta)],
             [0, 0.5*cos(theta)])
    plt.pause(0.001)

def main():
    plt.interactive(True)
    rospy.init_node('visualization', anonymous=True)
    rospy.Subscriber("pendulum_output", pendulum, callback,queue_size = 1)
    rospy.spin()
    #while not rospy.is_shutdown():
        


if __name__ == "__main__":
    main()
