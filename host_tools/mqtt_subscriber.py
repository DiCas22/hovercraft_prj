#!/usr/bin/env python3
import json
import paho.mqtt.client as mqtt

BROKER = "broker.hivemq.com"  # ou seu IP LAN
TOPIC  = "bia_kit/mpu"

def on_message(_c, _u, msg):
    try:
        data = json.loads(msg.payload)
        print(data)
    except Exception as e:
        print("raw:", msg.payload, "err:", e)

m = mqtt.Client()
m.connect(BROKER, 1883, 60)
m.subscribe(TOPIC)
m.on_message = on_message
m.loop_forever()
