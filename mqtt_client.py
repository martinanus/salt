import paho.mqtt.client as mqtt
import threading
import time
import datetime

mqttt_server = "salt-ma.cloud.shiftr.io"
mqtt_publisher_name = "Central Operativa"

mqtt_user_name = "salt-ma"
mqtt_user_token = "1f05DXKCezK1hnck"

# mqtt_user_name = "salt-ma2"
# mqtt_user_token = "F6EXqH8wRkBnjmn4"


mqtt_suscribe_log_topic = "salt_remote_log"
mqtt_suscribe_ack_topic = "salt_remote_ack"
mqtt_publish_topic = "salt_remote_command"


def on_connect(client, userdata, flags, reason_code, properties):
    client.subscribe(mqtt_suscribe_log_topic)
    client.subscribe(mqtt_suscribe_ack_topic)
    print(f"{mqtt_publisher_name} connected to server {mqttt_server}")


def on_message(client, userdata, msg):
    message = msg.payload.decode("utf-8")
    print(f"RX in {msg.topic}: {message}")

def publish_messages():
    while True:
        #timestamp = datetime.datetime.now()
        message = input()
        #message = f"{timestamp}: Comando enviado desde la Central Operativa"
        client.publish(mqtt_publish_topic, message)
        print(f"TX in {mqtt_publish_topic}: {message}")
        #time.sleep(5)  # Publish every 5 seconds


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, mqtt_publisher_name)
client.username_pw_set(mqtt_user_name, mqtt_user_token)

client.on_connect = on_connect
client.on_message = on_message

client.connect(mqttt_server, 1883, 60)

publisher_thread = threading.Thread(target=publish_messages)
publisher_thread.daemon = True
publisher_thread.start()

client.loop_forever()