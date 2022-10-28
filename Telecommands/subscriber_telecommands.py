# codigo de subscriptor MQTT (motor + camara)
# Equipo: UTIC
# Miembros:
#         - David Arias-Cachero Rincón
#         - Guillermo Lena Díaz
#         - Carmen Sánchez García

# =============================  CAMERA ===================================

from picamera import PiCamera
import time
import os

#Instanciamos la camara
camera = PiCamera()

#Contador de fotos
cont_photo = 0
#Contador de videos
cont_video = 0
#Nombre de la foto
name_photo = "foto_"
#Nombre del video
name_video = "video_"
#Comandos válidos para la camara
# c -> Snapshot
# v -> Start Recording
# sv -> Stop Recording
available_camera = ["c", "v", "sv"]

# =============================  CAMERA ===================================



# =============================  GPIO ===================================

#Librerias del uso para GPIO
import RPi.GPIO as GPIO          
from time import sleep

#Definicion de pines
#Motor 1
in1 = 22
in2 = 23
en_a = 27

#Motor 2
in3 = 24
in4 = 25
en_b = 26

#Variables temporales de cada motor
temp1 = 1
temp2 = 1

#Comandos válidos para los motores
available_motor = ["r", "f", "b", "s","d", "i", "l", "m", "h", "e"]


#Modo de trabajo GPIO
GPIO.setmode(GPIO.BCM)
#Defino los pines como salida
#Motor 1
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en_a,GPIO.OUT)
#Motor 2
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en_b,GPIO.OUT)

#Inicializo pines a nivel bajo
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)

#Genero PWM en el pin enable A (motor 1)
pwm_motor1=GPIO.PWM(en_a,1000)
pwm_motor1.start(25)

#Genero PWM en el pin enable B (motor 2)
pwm_motor2=GPIO.PWM(en_b,1000)
pwm_motor2.start(25)


print("\n")
print("La velocidad predeterminada es LOW y la direccion es FORWARD")
print("Comandos:\n")
print("Direccion:\n")
print("r-run s-stop f-forward b-backward")
print("Velocidad:\n")
print("l-low m-medium h-high e-exit")
print("\n")    

# =============================  GPIO ===================================




#=========================== MQTT ======================================

#Librerias para el uso de MQTT
import random
from paho.mqtt import client as mqtt_client

#Direccion del broker
broker_address = "192.168.1.230"
#Puerto del broker
broker_port = 1883
#Topico del motor
topic_motor = "motor"
#Topico de la camara
topic_camera = "camera"
#Topico usado
topic = ""
#Lista de topicos a los que me subscribo
topic_list = [topic_motor,topic_camera]

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
      
      #Hago global la variable temp1 para poder cambiar los estados del motor
      global temp1, temp2, topic, cont_photo, cont_video
      #El comando es el payload del mensaje
      comando = msg.payload.decode()
      print(f"Comando recibido: `{comando}`") 
      topic = msg.topic
      print(f"Topico: `{msg.topic}`")
     
     #Segun el tópico actuamos:
     #TOPICO DEL MOTOR
      if topic == topic_motor:

       #Caso1. Ponemos el motor a funcionar
        if comando=='r':
          print("run")
          if(temp1==1):
          #Para avanzar, giro en sentido horario (IN1 activo, IN2 NO activo)
            GPIO.output(in1,GPIO.HIGH)
            GPIO.output(in2,GPIO.LOW)
            GPIO.output(in3,GPIO.HIGH)
            GPIO.output(in4, GPIO.LOW)
            print("forward")
            comando = 'z'
          else:
          #Para retroceder, giro en sentido antihorario (IN1 NO activo, IN2 activo)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3, GPIO.LOW)
            GPIO.output(in4, GPIO.HIGH)
            print("backward")
            comando ='z'


        #Caso2. Stop
        elif comando =='s':
            print("stop")
            #Hay STOP cuando los dos IN están a LOW
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.LOW)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.LOW)
            comando ='z'

        #Caso3. Forward
        elif comando =='f':
            print("forward")
            GPIO.output(in1,GPIO.HIGH)
            GPIO.output(in2,GPIO.LOW)
            GPIO.output(in3,GPIO.HIGH)
            GPIO.output(in4,GPIO.LOW)
            temp1 = 1
            temp2 = 1
            comando ='z'

        #Caso4. Backward
        elif comando =='b':
            print("backward")
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)
            temp1 = 0
            temp2 = 0
            comando ='z'

        #Caso izquierda (rueda izq forward, rueda dcha backward)
        elif comando == 'i':
            print("left")
            GPIO.output(in1,GPIO.HIGH)
            GPIO.output(in2,GPIO.LOW)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)

        #Caso derecha (rueda izq backward, rueda dcha forward)
        elif comando == 'd':
            print("right")
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.HIGH)
            GPIO.output(in4,GPIO.LOW)
            comando = 'z'

        #Caso de velocidad LOW
        elif comando =='l':
            print("low")
            #Para menos velocidad, reduzco el DutyCycle
            pwm_motor1.ChangeDutyCycle(25)
            pwm_motor2.ChangeDutyCycle(25)
            comando ='z'

        elif comando =='m':
            print("medium")
            pwm_motor1.ChangeDutyCycle(50)
            pwm_motor2.ChangeDutyCycle(50)
            comando ='z'

        elif comando =='h':
            print("high")
            pwm_motor1.ChangeDutyCycle(75)
            pwm_motor2.ChangeDutyCycle(75)
            comando ='z'
       
        elif comando =='e':
            GPIO.cleanup()
            return
      
        else:
            print("<<<  wrong data  >>>")
            print("please enter the defined data to continue.....")

      
      #TOPICO DE CAMARA
      elif topic == topic_camera:
        #Esperamos
        time.sleep(2)

        #Comando de hacer foto
        if comando == "c":
          cont_photo = cont_photo + 1
          #Nombre de la foto
          filename = name_photo + str(cont_photo) + ".jpg"
          #Capturamos la foto
          camera.capture("pics/"+filename)
          print("Captured!")
          print("File: " + filename)

        #Comando de empezar video
        elif comando == "v":
          cont_video = cont_video + 1
          camera.resolution = (640, 480)
          #camera.framerate=25
          filename = name_video + str(cont_video)
          filename_h264 = filename + ".h264"
          filename_mp4 = filename + ".mp4"
          camera.start_recording(filename)
          print("Recording started!")

        #Comando de dejar de grabar
        elif comando == "sv":
          camera.stop_recording()
          print("Recording ended!")
          os.system("ffmpeg -loglevel quiet -framerate 24 -i -{0} -c copy -{1}").format(filename_h264,filename_mp4)
          os.system("sudo mv -t vids" + filename_h264 + " " + filename_mp4)
          print("Converted!")
         


    #Nos suscribimos al topico
    for topico in topic_list:

      client.subscribe(topico)
      print("Subscribed to topic: " + topico)
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
