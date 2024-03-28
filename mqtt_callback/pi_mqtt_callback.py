import paho.mqtt.client as mqtt
import random
import os

# MQTT Connection information
BROKER_ADDRESS = '192.168.68.182'
PORT = 1883
USERNAME = 'emqx'
PASSWORD = 'public'
TOPIC = 'FromRobot'
CLIENT_ID = 'python-mqtt-' + str(random.randint(0, 1000))

# File path to write the position data
FILE_PATH = '/home/pi/pos.txt'

# MQTT Client setup
client = mqtt.Client(CLIENT_ID)
client.username_pw_set(USERNAME, PASSWORD)

# Callback when connecting to the MQTT broker
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(TOPIC)

# Callback when receiving a message from the subscribed topic
def on_message(client, userdata, msg):
    message = msg.payload.decode('utf-8')
    if message.startswith('C3@SlamPose='):
        data = message.replace('C3@SlamPose=', '').split(':')
        if len(data) == 3:
            x, y, angle = data
            with open(FILE_PATH, 'w') as file:
                file_content = 'X:{}\nY:{}\nD:{}'.format(x, y, angle)
                file.write(file_content)
            print("Position data written to pos.txt")

# Assign event callbacks
client.on_connect = on_connect
client.on_message = on_message

# Connect to the broker and start the loop
client.connect(BROKER_ADDRESS, PORT, 60)
client.loop_forever()
