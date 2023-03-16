from paho.mqtt import client as paho
import json
import os
def on_message (client, userdata, msg):
	msg_decode = msg.payload.decode("utf-8", "ignore")
	print("msg_decode: ", msg_decode)
	
	msg_dict = json.loads(msg_decode)
	print("msg_dict: ", msg_dict)
	os.system("python3 testing.py")



if __name__ == '__main__':
	port = 1883
	topic = "/magni_robot/command"
	address = "ia.ic.polyu.edu.hk"
	client = paho.Client()
	client.connect(address, port, 60)
	client.subscribe(topic)
	client.on_message = on_message
	client.loop_start()
	while True:
		pass
