#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


def callbackJoy(msg):
    global speedX
    global speedX_1
    global yaw

    ### Read of b_button for stop the model-car
    global stop
    global go
    global b_stop
    global rightStickY
    global rightStickX
    global joys
    ### Red button for stop of model-car
    stop = msg.buttons[1]
    go = msg.buttons[0]
    if stop == 1:
        b_stop=True

    elif go == 1:
        b_stop=False



def main():

    global b_stop
    stop = 0
    speedX = 0
    speedX_1 = 0
    yaw = 0
    b_stop=False
    joys = False

    msgSpeed = Int16()
    msgSteering = Int16()
    msgStop = Int16()

    print "INITIALIZING JOYSTICK NODE FOR TELEOPERATION OF Toretto CAR... :)"
    rospy.init_node("joystick_teleop")

    rospy.Subscriber("/joy", Joy, callbackJoy)

    pubStop = rospy.Publisher("/manual_control/stop", Int16, queue_size=1)

    loop = rospy.Rate(10)

    while not rospy.is_shutdown():

        if b_stop:
            print "Stop"
            msgStop.data= 1
            pubStop.publish(msgStop)
            #b_stop=False
        else:
            msgStop.data= 0
            pubStop.publish(msgStop)


        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
