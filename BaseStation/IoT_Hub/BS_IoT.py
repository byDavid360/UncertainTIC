""" 
    Código en el que se implementa la app de la Base Station    
    Contiene:
        1) IoT Hub
        2) Sistema de archivos

    Fecha de ultima edicion: 9/12/2022

"""
#Librerias para implementar la GUI

#Librerias para la implementacion del MQTT
from paho.mqtt import client as mqtt_client
import random
#Libreria de hilos
from threading import Thread

# Librerias para IoT Hub
import time
import os
import json
from azure.iot.device import IoTHubDeviceClient, Message
import queue
# Librería timestamp
from datetime import datetime


############ Variables locales
rx_data4IoT = ""
directory = "./Logs/"
topicoMQTT = ""


#==================================================================================================
#=============================================== MQTT =============================================
#==================================================================================================
#IP del broker MQTT (la BS)
broker = "192.168.0.230"
port = 1883
#Topicos de pub
list_topic_pub = ["motor", "camera", "BS2gnuradio"]
#Topico al que se suscribe (el housekeeping)
list_topic_sub = ["housekeeping", "gnuradio2BS_DoA", "gnuradio2BS_DistEst", "emergency", "MR_status"]
#Generamos un client ID random
client_id = f'python-mqtt-{random.randint(0, 100)}'
#Usuario y passwd de MQTT
username = "utic"
password = "123456"

#Variable para guardar el comando de movimiento
cmd = ""
#Flag para comprobar si se ha clicado un boton
clicked = 0
#Comandos admitidos para los motores
lista_cmd_motores = ["run","forward","backward","stop","right","left","rotar_right","rotar_left","low","med","high"]
#Variable para el boton de RUN/STOP
cmd_stop_run = "run"
#Comandos admitidos para GNURADIO
lista_cmd_gnuradio = ["start_gnu", "stop_gnu"]
#Comandos admitidos para la cámara
lista_cmd_camera = ["photo", "start_record", "stop_record", "satar_live", "stop_live"]
#Variable para el boton de START/STOP Video
cmd_video = "start_record"
#Variable para el LIVE VIDEO
cmd_live = "start_live"

#Funcion MQTT para conectarnos al broker
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc==0:
            print("Successfully connected to MQTT broker")
        else:
            print("Failed to connect, return code %d", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

#Función para subscribirnos al MR
def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
    #print(f"Recieved '{msg.payload.decode()}' from '{msg.topic}' topic")
        global rx_data4IoT
        global topicoMQTT
        rx_data = msg.payload.decode()
        # rx_data4IoT.put(rx_data)
        #Estudio el topico
        topic = msg.topic
        if topic == "housekeeping":
            rx_data4IoT = rx_data
        #Si es housekeeping...
        topicoMQTT = topic 
        # print("RX from MR: " + rx_data)
        
    #Nos suscribimos a la lista de topicos
    for topico in list_topic_sub:
      client.subscribe(topico)
      print("Subscribed to topic: " + topico)

    client.on_message = on_message


 #Funcion para publicar en el broker
def publish(client):
    global cmd, clicked, cmd_stop_run
    #Topico usado al publicar (hay varios)
    topic = ""
    #Si se ha clicado un boton
    if (clicked != 0):
        #Los comandos de movimiento, se usa topico motor
        if cmd in lista_cmd_motores and cmd not in lista_cmd_gnuradio:
            #El topico usado es el de motor
            topic = list_topic_pub[0]
            #Enviamos el comando
            result = client.publish(topic, cmd)
        
        #Comando de cámara, se usa topico camara 
        elif cmd in lista_cmd_camera:
            #El topico usado es el de camara
            topic = list_topic_pub[1]
            result = client.publish(topic, cmd)
        #Estado de la publicacion

        #Caso de comandos para GNURADIO
        elif cmd in lista_cmd_gnuradio:
            topic = list_topic_pub[2]
            result = client.publish(topic,cmd)
        status = result[0]

        #Comprobamos
        if status == 0:
                    print(f"Success TX telecommand")
                    print("Topic: " + topic)
        else:
            if topic != "":
                print(f"Failed to send message to topic {topic}")
        
        #Limpiamos el flag de click
        clicked = 0


""" #Funcion para gestionar el comando desde los botones
def handle_btn(value):
    global cmd, clicked, cmd_stop_run, cmd_video

    #Boton RUN/STOP
    if value in lista_cmd_motores: 
        #Boton RUN STOP (cambia cuando se pulsa)
        if value == lista_cmd_motores[0]:
            btn_stop["text"] = "STOP"
            cmd_stop_run = lista_cmd_motores[3]
        
        
        elif value == lista_cmd_motores[3]:
            btn_stop["text"] = "RUN"
            cmd_stop_run = lista_cmd_motores[0]
        
        else:
            btn_stop["text"] = "STOP"
            cmd_stop_run = lista_cmd_motores[3]

    #Boton STAR/STOP VIDEO
    elif value in lista_cmd_camera:
        #Envio start_record
        if value == lista_cmd_camera[1]:
            #Lo siguiente a enviar seria stop_record
            cmd_video = lista_cmd_camera[2]

        #Envio stop_record
        elif value == lista_cmd_camera[2]:
            #Lo siguiente a enviar seria start_record
            cmd_video = lista_cmd_camera[1]



    print("\nCOMMAND: " + value)
    #El valor de movement y el flag clicked son el comando
    cmd = value
    clicked = value """

#==================================================================================================
#=============================================== MQTT =============================================
#==================================================================================================


#==================================================================================================
#===============================================IoT HUB============================================
#==================================================================================================

CONNECTION_STRING = "HostName=U-TIC-HubIoT.azure-devices.net;DeviceId=U-TIC-BS;SharedAccessKey=QHDuWpDilDxT1Bd8jr99td2jdWBpVxNg7Aniz9yKm4I="

def iothub_BS_init():
    # Create an IoT Hub client
    client = IoTHubDeviceClient.create_from_connection_string(CONNECTION_STRING)
    return client

def iothub_client_telemetry_sample_run(directory):
    try:
        client = iothub_BS_init()
        print ( "IoT Hub device sending periodic messages, press Ctrl-C to exit" )
        lineOld = ""
        directory = "./Logs/"                     ###TODO PONER PATH DONDE SE GUARDARAN LOS ARCHIVOS (acabado en /)
        t_ficheroMin = 5 # minutos ~ 150 lineas (+1)  # rx_data
        t_fichero = t_ficheroMin
        fileName = "data_sensor_rx.txt"
        os.system("sudo rm -r "+ directory + "*")
        # Guardar los datos en un .txt
        now = datetime.now()
        t_str = now.strftime("%d_%m_%Y %H_%M_%S")
        t_strSplit = t_str.split(" ")
        fileNameData = directory+t_strSplit[0]+"_"+t_strSplit[1]+"_"+fileName
        fichero = open(fileNameData, "w")
        fichero.write("ID MR;h min seg;ACC ACCX ACCY ACCZ m/s^2;GGY GGYX GGYY GGYZ rad/s;DIS D6 D7 [mm];Colision D6(Bool) D7(Bool);Temperatura Ambiental CPU_MR [ºC]\n")
        
        while True:
            # lineNew = rx_data4IoT.get()
            lineNew = rx_data4IoT
            if lineNew != lineOld: # and topicoMQTT == "housekeeping":
                lineOld = lineNew
                # print(lineNew)
                    # Porceso los datos
                lineSplit = lineNew.split(";")
                ACCval = lineSplit[2].split(" ")
                GGYval = lineSplit[3].split(" ")
                DISval = lineSplit[4].split(" ")
                ColisionVal = lineSplit[5].split(" ")
                Temperatura = lineSplit[6].split(" ")
                data_time = lineSplit[1].split(" ")
                x = {
                    "AACX": float(ACCval[0]),
                    "AACY": float(ACCval[1]),
                    "ACCZ": float(ACCval[2]),
                    "GGYX": float(GGYval[0]),
                    "GGYY": float(GGYval[1]),
                    "GGYZ": float(GGYval[2]),
                    "D6": float(DISval[0]),
                    "D7": float(DISval[1]),
                    "Ambiental": float(Temperatura[0]),
                    "RaspberryTemp": float(Temperatura[1]),
                    "D6Colision": float(ColisionVal[0]),
                    "D7Colision": float(ColisionVal[1])
                    }
                        # convert into JSON string:
                msg_txt_formatted = json.dumps(x)
                message = Message(msg_txt_formatted)
                        # Send the message.
                print( "Sending message: {}".format(message) )
                client.send_message(message)
                print ( "Message successfully sent" )
                time.sleep(1.5)

                # Guardar los datos en un .txt
                t_ficheroReal = data_time[1]
                if t_ficheroReal == t_ficheroMin:
                    t_ficheroMin = t_ficheroMin + t_fichero
                    fichero.close()
                    t_str = now.strftime("%d_%m_%Y %H_%M_%S")
                    t_strSplit = t_str.split(" ")
                    fileNameData = directory+t_strSplit[0]+"_"+t_strSplit[1]+"_"+fileName
                    fichero = open(fileNameData, "w")
                    fichero.write("ID MR;h min seg;ACC ACCX ACCY ACCZ m/s^2;GGY GGYX GGYY GGYZ rad/s;DIS D6 D7 [mm];Colision D6(Bool) D7(Bool);Temperatura Ambiental CPU_MR [ºC]\n")
                else:
                    fichero.write(lineNew)
                    fichero.write("\n")                            ### TODO Le has quitado el salto de linea al final?
                    
                            
    except KeyboardInterrupt:
        print ( "U-TIC_BS sample stopped" )
#==================================================================================================
#===============================================IoT HUB============================================
#==================================================================================================


def run():
    #Nos conectamos al broker
    # Call work function
    client = connect_mqtt()
    client.loop_start()
    subscribe(client)
    while True:
        publish(client)

def runIoT():
    iothub_client_telemetry_sample_run(directory)
    
   
if __name__ == '__main__':
    #Hacemos un thread para la funcion run y que esté en paralelo con el bucle infinito del Hub IoT
    t1=Thread(target=run)
    t1.start()
    t2=Thread(target=runIoT)
    t2.start()
