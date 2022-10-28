
""" Código de telecomandos via MQTT (publicador)

    Equipo: UTIC
    Miembros: 
            - David Arias-Cachero Rincón
            - Guillermo Lena Díaz
            - Carmen Sánchez García


"""

#We import the client
from paho.mqtt import client as mqtt_client
import time
import random

#We define the broker (robot IP), port and topic
# IMPORTANT: by now, the broker is the robotc but in the future it'll the base station
#IP Broker MR: 192.168.1.135
#IP Broker BS: 192.168.1.230
broker = "192.168.1.230"
port = 1883
#Topico
topic_motor = "motor"
#Topico de camara
topic_camera = "camera"


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
        
        #Esperamos 1 segundo antes de salir del on_connect
        time.sleep(1)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username,passwd)
    client.on_connect = on_connect
    client.connect(broker,port)
    return client


#MQTT Publish function
def publish(client):

    """TELECOMMANDS
        Move Forward -> f
        Move Backward -> b
        Stop -> s
        Low Speed -> l
        Medium Speed -> m
        High Speed -> h
        Exit -> e
        Snapshot -> c
        Start Video -> v
        Stop Video -> sv
    """
    global topic
    available_tele = ["r", "f", "b", "s","d", "i", "l", "m", "h", "e", "c","v","sv"]
    command_camera = ["c","v","sv"]
    telecom = input("\nCommand:")

    if (telecom in available_tele):
        
        print ("Telecommand TX: " + telecom)
        #Si el telecomando NO es el de cámara (c), enviamos a topico motor
        if telecom not in  command_camera:
            topic = topic_motor
            msg = telecom
            result = client.publish(topic, msg)

        #Si el telecomando SI es el de cámara, enviamos a tópico de cámara
        elif telecom in command_camera:
            topic = topic_camera
            msg = telecom
            result = client.publish(topic,msg)

        #Estado de la publicacion
        status = result[0]
    else:
        print("Comando INVALIDO.")
        print("Utilice uno de los siguientes comandos:")
        print("Move Forward -> f\nMove Backward -> b\nStop -> s\nLow Speed -> l\nMedium Speed -> m\nHigh Speed -> h\nSnapshot -> c\nExit -> e")
        status = ""

    if status == 0:
                print(f"Success TX telecommand")
                print("Topic: " + topic)
    else:
        if topic != "":
            print(f"Failed to send message to topic {topic}")


    #Reinicio el tópico
    topic = ""

def run():
    client = connect_mqtt()
    client.loop_start()
    while True:
        publish(client)


if __name__ == '__main__':
    run()