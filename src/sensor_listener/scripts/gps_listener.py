#!/usr/bin/env python
# license removed for brevity
import rospy, math
from std_msgs.msg import String
import json
import paho.mqtt.client as mqtt
from sensor_msgs.msg import NavSatFix, NavSatStatus


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("sensors/gps")

def on_message(client, userdata, msg):
    global pub
    y = json.loads(msg.payload)

    navsat = NavSatFix()
    navsat.header.stamp = rospy.Time.now()
    navsat.header.frame_id = 'gps'
    navsat.status.status = 0
    navsat.status.service = 1

    latitude = y["latitude"]
    longitude = y["longitude"]
    navsat.latitude = float(latitude[:2])+float(latitude[2:])/60
    navsat.longitude = float(longitude[:3])+float(longitude[3:])/60

    if y["lat"] == "S":
        navsat.latitude = -1 * navsat.latitude
    if y["lon"] == "W":
        navsat.longitude = -1 * navsat.longitude

    navsat.altitude = y["altitude"]
    pub.publish(navsat)


if __name__ == '__main__':
    print("starting satnav listener")
    pub = rospy.Publisher('fix', NavSatFix, queue_size=10)
    rospy.init_node('imu_listener', anonymous=True)
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect("localhost", 1883, 60)
    client.loop_forever()

