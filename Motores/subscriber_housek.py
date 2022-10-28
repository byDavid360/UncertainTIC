"""Código de subscriber para recibir la temperatura del MR"""

import random
from paho.mqtt import client as mqtt_client

#Direccion del broker
broker_address = "192.168.1.230"
#Puerto del broker
broker_port = 1883
#Topico
topic = "housekeeping"

#We define the username and passwd according to the configuration of the MQTT broker
username = "utic"
passwd = "123456"

# Generamos un ID de cliente
client_id = f'python-mqtt-{random.randint(0, 100)}'

#Funcion de conexion al broker
def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, passwd)
    client.on_connect = on_connect
    client.connect(broker_address, broker_port)
    return client



def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
      
     
      #El comando es el payload del mensaje
      temp = msg.payload.decode()
      print(f"MR's internal temperature: {temp} ºC") 
  

    #Nos suscribimos al topico
    client.subscribe(topic)
    client.on_message = on_message


#Funcion de start
def run():
    #Nos conectamos
    client = connect_mqtt()
    #Nos suscrubimos
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()


 #=========================== MQTT ======================================
