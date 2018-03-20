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
    global b_stop
    global rightStickY
    global rightStickX
    global joys
    ### Red button for stop of model-car
    stop = msg.buttons[1]
    y = msg.buttons[3]
    x = msg.buttons[2]

    ### Control of model-car with right Stick
    rightStickX = msg.axes[1]
    rightStickY = msg.axes[3] 

    #print "axes 1: " + str(msg.axes[1])
    speedX_1=speedX

    magnitudRight = math.sqrt(rightStickX*rightStickX + rightStickY*rightStickY)
    if magnitudRight > 0.1:
        speedX = rightStickX
        yaw = rightStickY
    else:
        speedX = 0
        yaw = 0

    if stop == 1:
        b_stop=True    

    else:
        b_stop=False 


    if y == 1:
        joys=True
    if x == 1:
        joys=False    

def main():

    global speedX
    global speedX_1
    global yaw
    global stop
    global b_stop
    global rightStickY
    global rightStickX
    global joys

    stop = 0
    speedX = 0
    speedX_1 = 0
    yaw = 0
    b_stop=False
    joys = False

    msgSpeed = Int16()
    msgSteering = Int16()
    msgStop = Int16()

    print "INITIALIZING JOYSTICK NODE FOR TELEOPERATION MODEL CAR... :)"
    rospy.init_node("joystick_teleop")
       
    rospy.Subscriber("/joy", Joy, callbackJoy)

    pubSpeed = rospy.Publisher("/manual_control/speed", Int16, queue_size=1)
    pubSteering = rospy.Publisher("/manual_control/steering", Int16, queue_size=1)
    pubStop = rospy.Publisher("/manual_control/stop", Int16, queue_size=1)

    loop = rospy.Rate(10)

    while not rospy.is_shutdown():

        if (joys):
                if math.fabs(speedX) > 0:
                    msgSpeed.data = int(speedX*-1500)
                    speedX= ((speedX - speedX_1)*0.3)+speedX_1
                    print "speed_ x: " + str(msgSpeed) 
                else:
                    rightStickX = 0
                    msgSpeed.data = 0
                    print "Speed_x: " +  str(msgSpeed)

                if math.fabs(yaw) > 0:
                	msgSteering.data = int(((yaw)*170)+120)
                	print "Steering: " + str(msgSteering)
                else:
                    msgSteering.data = 120
                    print "Steering: " + str(msgSteering)
                    #rightStickY = 0
                    

                pubSteering.publish(msgSteering)
                pubSpeed.publish(msgSpeed)

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
