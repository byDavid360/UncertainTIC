"""
Código del publicador de Housekeeping

SUBSCRIBER: BASE STATION OR LAPTOR
PUBLISHER: MR

En este código se implementa el subscriptor para housekeeping
Se recibe de forma automática cada segundo el valor de la temperatura interna del MR
El broker está ubicado en la raspberry de la BS


"""
import os
import time
import numpy as np


#Lista para almacenar las samples de temperatura
lista_temps = []
#Contador de samples
cont = 0

from paho.mqtt import client as mqtt_client
import time
import random


broker = "192.168.1.230"
port = 1883
#Topico
topic = "housekeeping"

#We define the username and passwd according to the configuration of the MQTT broker
username = "utic"
passwd = "123456"
#Random generation of a mqtt client id
client_id = f'python-mqtt-{random.randint(0, 1000)}'

#MQTT Conection function
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Se ha conectado al broker")
        else:
            print("Fallo de conexion. CODE: %d\n", rc)
    client = mqtt_client.Client(client_id)
    client.username_pw_set(username,passwd)
    client.on_connect = on_connect
    client.connect(broker,port)
    return client



def publish(client):
  
  #Hacemos global la variable del contador de samples
  global cont

  while True:  
    #Cada 10 samples
    while cont<10:
      #Guardo el output del comando en una variable
      output = os.popen('vcgencmd measure_temp').read()
      #El output es de la forma temp = x.x'C
      #Hago split en el = y me quedo solo con los grados
      #que son el segundo elemento del array obetnido tras el split
      temp = output.lstrip("temp = ").rstrip("'C\n")
      #print("Temperatura actual:", temp)
      lista_temps.append(float(temp))
      cont = cont + 1
      print("temp: ", temp)
      print("cont = ", cont)
      print(lista_temps)
    #Hacemos la media (redondeamos a 2 decimales)
    media =round(np.mean(lista_temps),2)
    print("Temperatura media: ", media, " ºC" )
    #Reseteamos el contador y vaciamos la lista
    cont = 0
    lista_temps.clear()
    msg = media
    result = client.publish(topic,msg)
    print("Dato enviado: ", msg)
    #Estado de la publicacion
    status = result[0]
    if status == 0:
      print(f"Success TX telecommand")
      print("Topic: " + topic)
    else:
      print(f"Failed to send message to topic {topic}")
  
    print("\n")
    time.sleep(1)

def run():
    client = connect_mqtt()
    client.loop_start()
    while True:
        publish(client)


if __name__ == '__main__':
    run()
