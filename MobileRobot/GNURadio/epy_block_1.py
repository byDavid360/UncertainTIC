"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
from gnuradio import gr

#Librerias para GPIO
import RPi.GPIO as GPIO          

from time import sleep
import time

import random
import paho.mqtt.client as mqtt_client
from statistics import mode
import os


class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block

    def __init__(self, nsamples = 20e3, comparation_level = 1, tolerance = 1.3):  # only default arguments here
        gr.sync_block.__init__(
            self,
            name='DoA + Dist Est',   # will show up in GRC
            in_sig=[np.float32, np.float32, np.float32],
            out_sig=[np.float32]
        )
        
        # ----- Block self variables ------------------
        self.counter = 0
        self.nsamples = nsamples
        self.min = comparation_level - comparation_level*tolerance
        self.max = comparation_level + comparation_level*tolerance
        self.dist_est_flag = False
        self.dist_estimation = []
        self.do_stuff = True
        self.safe_to_go = True
        self.ignored_samps = 0
        self.decision = []
        self.straight_counter = 0
        # ----------------------------------------
        
        
        # ----- Motores GPIO ---------------------
        #Motor 1
        self.in1 = 22
        self.in2 = 23
        self.en_a = 27
        #Motor 2
        self.in3 = 24
        self.in4 = 25
        self.en_b = 26
        
        GPIO.setwarnings(False)
        #Modo de trabajo GPIO
        GPIO.setmode(GPIO.BCM)
        #Defino los pines como salida
        #Motor 1
        GPIO.setup(self.in1,GPIO.OUT)
        GPIO.setup(self.in2,GPIO.OUT)
        GPIO.setup(self.en_a,GPIO.OUT)
        #Motor 2
        GPIO.setup(self.in3,GPIO.OUT)
        GPIO.setup(self.in4,GPIO.OUT)
        GPIO.setup(self.en_b,GPIO.OUT)

        #Inicializo pines a nivel bajo para q este parado
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        GPIO.output(self.in3,GPIO.LOW)
        GPIO.output(self.in4,GPIO.LOW)
        
        # Genero PWM en el pin enable A (motor 1)
        self.pwm_motor1=GPIO.PWM(self.en_a,800)
        self.pwm_motor1.start(10)

        # Genero PWM en el pin enable B (motor 2)
        self.pwm_motor2=GPIO.PWM(self.en_b,800)
        self.pwm_motor2.start(10)
        # ----------------------------------------
        
        
        # ----- MQTT -----------------------------
        #Direccion del broker
        self.broker_address = '192.168.0.230'
        #Puerto del broker
        self.broker_port = 1883
        #Topicos de PUBLICACION
        self.topic_pub_DoA = "gnuradio2BS_DoA"
        self.topic_pub_DistEst = "gnuradio2BS_DistEst"
        #Topicos de SUBSCRIPCION
        self.topic_sub = "BS2gnuradio"
        #We define the username and passwd according to the configuration of the MQTT broker
        self.username = "utic"
        self.passwd = "123456"
        # Generamos un ID de cliente
        self.client_id = f'python-mqtt-{random.randint(0, 100)}'
        #Nos conectamos
        self.client = self.connect_mqtt()
        self.client.loop_start()
        #self.subscribe(self.client)
        print('despues de subscribe()')
        # ----------------------------------------
        
    #Funcion de conexion al broker
    def connect_mqtt(self) -> mqtt_client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!\n")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mqtt_client.Client(self.client_id)
        client.username_pw_set(self.username, self.passwd)
        client.on_connect = on_connect
        client.connect(self.broker_address, self.broker_port)
        return client

    
    #Funcion de publicar al broker
    def publish(self, client, msg, topic):
        result = client.publish(topic,msg)
        status = result[0]
        if status == 0:
          print("Success publishing ...")
          print("Topic: " + topic)
        else:
          print("Failed to send message to topic " + topic)
        print("\n")


    def work(self, input_items, output_items):
                
        if self.do_stuff and self.safe_to_go:
            if (not self.dist_est_flag):
                for i in range(len(input_items[0])):
                    if self.counter >= self.nsamples:
                        self.counter = 0
                        
                        if (input_items[0][i] >= self.min) and (input_items[0][i] <= self.max):
                            self.decision.append('S')
                        
                        elif input_items[0][i] > self.max:
                            self.decision.append('R')
                            
                        else:
                            self.decision.append('L')
                            
                        if len(self.decision) == 100:
                            final_decision = mode(self.decision)
                            print("Decision de DoA: ", final_decision)
                            
                            if final_decision == 'S':
                                GPIO.output(self.in1,GPIO.LOW)
                                GPIO.output(self.in2,GPIO.LOW)
                                GPIO.output(self.in3,GPIO.LOW)
                                GPIO.output(self.in4,GPIO.LOW)
                                
                                self.straight_counter+=1
                                
                                self.publish(self.client, 'Looking straight to BS ', self.topic_pub_DoA)
                                
                                if self.straight_counter == 5:
                                    self.dist_est_flag = True
                                    
                                    self.publish(self.client, 'DoA finished ... changing to Distance Estimation mode ...', self.topic_pub_DoA)
                                    
                            elif final_decision == 'L':
                                GPIO.output(self.in1,GPIO.LOW)
                                GPIO.output(self.in2,GPIO.HIGH)
                                GPIO.output(self.in3,GPIO.HIGH)
                                GPIO.output(self.in4,GPIO.LOW)
                                
                                self.straight_counter = 0
                                
                                self.publish(self.client, 'Rotating to the left', self.topic_pub_DoA)
                                
                            else:
                                GPIO.output(self.in1,GPIO.HIGH)
                                GPIO.output(self.in2,GPIO.LOW)
                                GPIO.output(self.in3,GPIO.LOW)
                                GPIO.output(self.in4,GPIO.HIGH)
                                
                                self.straight_counter = 0
                                
                                self.publish(self.client, 'Rotating to the right', self.topic_pub_DoA)
                                
                            self.decision = []
                    self.counter+=1

            else:
                for i in range(len(input_items[2])):
                    if self.counter >= self.nsamples:
                        self.counter = 0
                        self.dist_estimation.append(input_items[2][i])
                        
                        if len(self.dist_estimation) == 100:
                            dist_estim_mean = np.mean(self.dist_estimation)
                            print(dist_estim_mean)
                            self.publish(self.client, str(dist_estim_mean), self.topic_pub_DistEst)
                            self.do_stuff = False
                            self.dist_est_flag = False
                            os.system('pkill -f DoADistEst.py')
                        
                    self.counter+=1
                
        output_items[0][:] = input_items[0]
        return len(output_items[0])
