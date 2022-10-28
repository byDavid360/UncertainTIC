#Código de subscriptor MQTT (motor)




# =============================  GPIO ===================================

#Librerias del uso para GPIO
import RPi.GPIO as GPIO          
from time import sleep

#Definicion de pines
in1 = 22
in2 = 23
en = 27
temp1=1

#Modo de trabajo GPIO
GPIO.setmode(GPIO.BCM)
#Defino los pines como salida
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)
#Los inicializo a nivel bajo
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
#Genero PWM en el pin enable
p=GPIO.PWM(en,1000)
p.start(25)


print("\n")
print("La velocidad predeterminada es LOW y la dirección es FORWARD")
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
broker_address = "192.168.1.135"
#Puerto del broker
broker_port = 1883
#Topico
topic = "motor"

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
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client



def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):

    	comando = msg.payload.decode()
        print(f"Received `{comando}` from `{msg.topic}` topic")

    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()
