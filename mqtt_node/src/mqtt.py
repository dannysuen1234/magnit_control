#!/usr/bin/env python
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import String
# from geometry_msgs.msg import PoseStamped, PoseArray, PolygonStamped, PoseWithCovarianceStamped, Pose

MQTT_SERVER = 'ia.ic.polyu.edu.hk'
MQTT_PORT = 1881
MQTT_CLIENT_ID = 'Pi4_001'
AGV_TOPIC_PUB = 'iot/agv'
IOT_TOPIC_SUB = 'iot/ctl'

# def amcl_pose_received(amclpose):
#     str = String()
#     str = '{}'

def mqtt_sub_received(message):
    print('mqtt pub:' + message.data)
    client.publish(AGV_TOPIC_PUB, message.data)

def on_message(client, userdata, message):
    print('mqtt sub' + message.payload)

def mqttclient():
    global client
    global mqtt_pub
    global mqtt_sub

    rospy.init_node('mqtt_client')
    mqtt_pub = rospy.Publisher("/mqtt_sub", String, queue_size=10)          
    mqtt_sub = rospy.Subscriber("/mqtt_pub", String, mqtt_sub_received)
    # amcl_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_received)

    client = mqtt.Client(MQTT_CLIENT_ID)
    client.connect(MQTT_SERVER, port=MQTT_PORT)
    client.on_message = on_message
    client.subscribe(IOT_TOPIC_SUB)
    
    # client.loop_start()
    # while True:
    #     pass
    client.loop_forever()

if __name__ == '__main__':
    try:
        #subprocess.call(['sudo', 'hciconfig', 'hci0', 'up'])
        mqttclient()
    except rospy.ROSInterruptException:
        pass
