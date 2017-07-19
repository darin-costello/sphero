#!/usr/bin/env python
# This file is modified from teleop code of turtlebot

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Sphero!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only x speed by 10%
e/c : increase/decrease only y speed by 10%
space key, k : force stop


CTRL-C to quit
"""

moveBindings = {
        'i':(0,20),
        'o':(20,20),
        'u':(-20,20),
        'l':(20,0),
        'j':(-20,0),
        ',':(0,-20),
        '.':(20,-20),
        'm':(-20,-20),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(x,y):
    return "current:\tx %s\ty %s " % (x,y)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('sphero_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    x = 0
    y = 0
    k_x = 1.0
    k_y = 1.0
    count = 0
    try:
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                k_x = k_x * speedBindings[key][0]
                k_x = k_x * speedBindings[key][1]
                

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0
                y = 0
                k_x = 1.0
                k_y = 1.0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    y = 0
                    k_x = 1.0
                    k_y = 1.0
                if key == '\x03':
                    break

            
            x_speed = k_x * x
            y_speed = k_y * y
            print vels(x_speed, y_speed)

            twist = Twist()
            twist.linear.x = x_speed; twist.linear.y = y_speed; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

