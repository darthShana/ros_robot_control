#!/usr/bin/env python
import rospy, math
from std_msgs.msg import String
import requests
import json
from geometry_msgs.msg import Twist


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)

def callback(msg):
    global wheelbase
    global client

    # rospy.loginfo("Received a /cmd_vel message!")
    # rospye.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    # rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    v = msg.linear.x

    steering = convert_trans_rot_vel_to_steering_angle(v, msg.angular.z, wheelbase)
    steering = math.degrees(steering)

    if v > 0.05 and v < 0.19 and abs(steering) > 25:
        v = 0.19
    if v < -0.05 and v > -0.21 and abs(steering) > 25:
        v = -0.21
    if v < 0:
        steering = steering * -1

    post_json=json.dumps({"velocity": v,"steering": steering});
    rospy.loginfo("message:"+post_json)
    headers={'Content-type':'application/json', 'Accept':'text/plain'}

    r = requests.post('http://192.168.4.1/cmdVel', data=post_json, headers=headers)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    pub = rospy.Publisher('chatter', String, queue_size=1)
    rospy.init_node('cmd_vel_listener', anonymous=True)

    rospy.Subscriber("cmd_vel", Twist, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    wheelbase = 0.35
    listener()
