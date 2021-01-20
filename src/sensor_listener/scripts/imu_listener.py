#!/usr/bin/env python
# license removed for brevity
import rospy, math
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Vector3
import json
import paho.mqtt.client as mqtt
from sensor_msgs.msg import Imu


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("sensors/imu")

def on_message(client, userdata, msg):
    global pub
    y = json.loads(msg.payload)

    imu = Imu()
    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = 'imu_link'

    imu.linear_acceleration.x = float(y["a.x"])
    imu.linear_acceleration.y = float(y["a.y"])
    imu.linear_acceleration.z = float(y["a.z"])

    imu.angular_velocity.x = float(y["v.x"])
    imu.angular_velocity.y = float(y["v.y"])
    imu.angular_velocity.z = float(y["v.z"])

    imu.orientation.w = float(y["o.w"])
    imu.orientation.x = float(y["o.x"])
    imu.orientation.y = float(y["o.y"])
    imu.orientation.z = float(y["o.z"])

    pub.publish(imu)


if __name__ == '__main__':
    print("starting imu listener")
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    rospy.init_node('imu_listener', anonymous=True)
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect("localhost", 1883, 60)
    client.loop_forever()

