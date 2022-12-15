""" 
    Código en el que se implementa la app de la Base Station    
    Contiene:
        1) Botones de control de motores
        2) Dashboard (parcialmente)

    Fecha de ultima edicion: 9/12/2022

"""
#Librerias para implementar la GUI
from tkinter import *
import tkinter as tk
#Librerias para la implementacion del MQTT
from paho.mqtt import client as mqtt_client
import random
#Libreria de hilos
from threading import Thread


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
#ID del MR
id_MR = ""

#Variable para guardar el comando de movimiento
cmd = ""
#Flag para comprobar si se ha clicado un boton
clicked = 0
#Comandos admitidos para los motores
lista_cmd_motores = ["run","forward","backward","stop","right","left","rotar_right","rotar_left","low","med","high"]
#Variable para el boton de RUN/STOP
cmd_stop_run = "run"
#Comandos admitidos para GNURADIO
lista_cmd_gnuradio = ["start_distest_L", "start_DoA", "start_distest_R"]
#Comandos admitidos para la cámara
lista_cmd_camera = ["photo", "start_record", "stop_record", "start_live", "stop_live"]
#Variable para el boton de START/STOP Video
cmd_video = "start_record"
#Variable para el LIVE VIDEO
cmd_live = "start_live"
#Variable que guarda el estado de nuestra cámara (photo,video o streaming)
camera_status = ""
#Flag para comprobar si estamos en live
flag_live = 0

#Lista para almacenar las alarmas
list_alerts = []




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


#Funcion para publicar en el broker
def publish(client):
    global cmd, clicked, cmd_stop_run, flag_live, camera_status
    result = []
    #Topico usado al publicar (hay varios)
    topic = ""
    #Si se ha clicado un boton
    if (clicked != 0): 
        #Comando a TX
        cmd_tx = cmd.split(";")[1]
        #Los comandos de movimiento, se usa topico motor
        if cmd_tx in lista_cmd_motores and cmd_tx not in lista_cmd_gnuradio:
            #El topico usado es el de motor
            topic = list_topic_pub[0]
            #Enviamos el comando
            result = client.publish(topic, cmd)
            print("se ha publicado " + cmd)
        
        #Comando de cámara, se usa topico camara 
        elif cmd_tx in lista_cmd_camera:
            #El topico usado es el de camara
            topic = list_topic_pub[1]
            if cmd_tx == "start_live":
                flag_live = 1
                result = client.publish(topic,cmd)
                camera_status == "stop_live"
            elif cmd_tx == "stop_live":
                flag_live = 0
                result = client.publish(topic, cmd)
                camera_status = "start_live"
            else:
                if flag_live == 0:
                    result = client.publish(topic, cmd)
                else:
                    status_MR_label.config(text = "Can't take photos or vidos while streaming...", fg="black")


        #Estado de la publicacion

        #Caso de comandos para GNURADIO
        elif cmd_tx in lista_cmd_gnuradio:
            topic = list_topic_pub[2]
            result = client.publish(topic,cmd)

        if bool(result)==True:
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


#Funcion para gestionar la cadena de datos recibida via MQTT
def data_handling(rx_data):
    #FORMATO DEL STRING QUE LLEGA
    # RESTRUCTURAR COMO SIGUE:
    #MR-id;h min seg;ACCX ACCY ACCZ;GGYX GGYY GGYZ;D6 D7;COLI_D6 COLI_D7;TEMP_AMBIENTE TEMP_CPU
    data = rx_data.split(";")
    #recojo el ID del MR que nos TX mensajes
    MR_id_rx = data[0]
    #La temperatura es el ultimo dato de la cadena
    #temp = data[len(data)-1]
    temp = data[6].split(" ")
    tempCPU = temp[1]
    tempExt = temp[0]
    #Hago split en la tercera componente de los datos (son las distancias)
    data_dist = data[4].split(" ")
    #La distancia 1 es la componente 1 de lo anterior y la distancia 2 la 2 ej: (DIS 245 231 mm)
    data_dist1 = data_dist[0]
    data_dist2 = data_dist[1]

    #La colision es la componente 4
    data_colision = data[5].split(" ")
    #La colision 1 es la componente 1 y la colision 2 la componente 2 ej: (COLISION 0 0)
    data_col1 = data_colision[0]
    data_col2 = data_colision[1]
    
    #El giroscopio es la componente 2
    data_giro = data[3].split(" ")
    #La componente X es la 1, la Y la 2 y la Z la 3 ej: (GGY X Y Z)
    data_giroX = data_giro[0]
    data_giroY = data_giro[1]
    data_giroZ = data_giro[2]

    #El acelerometro es la componente 1
    data_acc = data[2].split(" ")
    #La componente X es la 1, la Y la 2 y la Z la 3 ej: (GGY X Y Z)
    data_accX = data_acc[0]
    data_accY = data_acc[1]
    data_accZ = data_acc[2]

    #MIRAR SI SE PUEDE HACER DE OTRA FORMA
    if data_col1 == "1":
        data_col1 = "YES"
    else:
        data_col1 = "NO"

    if data_col2 == "1":
        data_col2 = "YES"
    else:
        data_col2 = "NO"
    
    #Diccionario para sacar cada uno de los datos
    dict_data = {"MR-ID": MR_id_rx,
                "tempExt": tempExt,
                 "tempCPU": tempCPU,
                 "dist1": data_dist1,
                 "dist2": data_dist2,
                 "col1": data_col1,
                 "col2": data_col2,
                 "giroX": data_giroX,
                 "giroY": data_giroY,
                 "giroZ": data_giroZ,
                 "accX": data_accX,
                 "accY": data_accY,
                 "accZ": data_accZ}
                 
    return dict_data


#Funcion para gestionar el streaming vs fotos/videos
def handle_video(cmd):
    global camera_status
    #Estamos transmitiendo en vivo
    if cmd == "start_live":
        print("Camera is streaming...")
        camera_status = cmd

    return camera_status
    #El resto de casos que no sean straming
    

#Funcion para tratar las alertas e imprimirlas en la dashboard
def handle_emergencies(rx_data):
    global list_alerts
    list_alerts.append(rx_data)

    #Primera alerta
    if len(list_alerts) == 1:
        alerta = list_alerts[0].split(";")
        id_emergency = alerta[0]
        descr_emergency = alerta[1]
        #Los escribo en la GUI
        emergency_id_label.config(text = id_emergency, fg="red")
        emergency_descr_label.config(text = descr_emergency, fg="red")
    
    #Segunda alerta
    elif len(list_alerts) == 2:
        alerta = list_alerts[1].split(";")
        id_emergency = alerta[0]
        descr_emergency = alerta[1]
        #Los escribo en la GUI
        emergency2_id_label.config(text = id_emergency, fg="red")
        emergency2_descr_label.config(text = descr_emergency, fg="red")
    
    #Tercera alerta
    elif len(list_alerts) == 3:
        alerta = list_alerts[2].split(";")
        id_emergency = alerta[0]
        descr_emergency = alerta[1]
        #Los escribo en la GUI
        emergency3_id_label.config(text = id_emergency, fg="red")
        emergency3_descr_label.config(text = descr_emergency, fg="red")

    #Cuarta alerta
    elif len(list_alerts) == 4:
        alerta = list_alerts[3].split(";")
        id_emergency = alerta[0]
        descr_emergency = alerta[1]
        #Los escribo en la GUI
        emergency4_id_label.config(text = id_emergency, fg="red")
        emergency4_descr_label.config(text = descr_emergency, fg="red")

    #Quinta alerta
    elif len(list_alerts) == 5:
        alerta = list_alerts[4].split(";")
        id_emergency = alerta[0]
        descr_emergency = alerta[1]
        #Los escribo en la GUI
        emergency5_id_label.config(text = id_emergency, fg="red")
        emergency5_descr_label.config(text = descr_emergency, fg="red")
    
    #Reiniciamos el guardado de alertas
    else:
        list_alerts = []
        #Borramos el texto de las labels
        emergency_id_label.config(text = "", fg="red")
        emergency_descr_label.config(text = "", fg="red")
        emergency2_id_label.config(text = "", fg="red")
        emergency2_descr_label.config(text = "", fg="red")
        emergency3_id_label.config(text = "", fg="red")
        emergency3_descr_label.config(text = "", fg="red")
    


#Función para subscribirnos al MR
def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        global list_alerts
    #print(f"Recieved '{msg.payload.decode()}' from '{msg.topic}' topic")
        rx_data = msg.payload.decode()
        #Estudio el topico
        topic = msg.topic
        #Si es housekeeping...
        if topic == list_topic_sub[0]:
            #Hago el data handling de lo RX en MQTT 
            dict_data = data_handling(rx_data)
            if dict_data["MR-ID"] == id_MR:
                #Cambio el valor en los labels para que muestren los datos 
                
                cpu_temp_label.config(text=dict_data["tempCPU"]+" °C",fg="black")
                ext_temp_label.config(text=dict_data["tempExt"]+" °C",fg="black")
                dist_rear_label.config(text = dict_data["dist1"] + " mm", fg="black")
                dist_front_label.config(text = dict_data["dist2"] + " mm", fg="black")
                coli_rear_label.config(text = dict_data["col1"], fg="black")
                coli_front_label.config(text = dict_data["col2"], fg="black")
                giroX_label.config(text = dict_data["giroX"] + " rad/s", fg="black")
                giroY_label.config(text = dict_data["giroY"] + " rad/s", fg="black")
                giroZ_label.config(text = dict_data["giroZ"] + " rad/s", fg="black")
                accX_label.config(text = dict_data["accX"] + " m/s^2", fg="black")
                accY_label.config(text = dict_data["accY"] + " m/s^2", fg="black")
                accZ_label.config(text = dict_data["accZ"] + " m/s^2", fg="black")
            

        #Si es gnuradio2BS_DoA...
        elif topic == list_topic_sub[1]:
            #Escribo los valores en el dashboard
            DoA_label.config(text = rx_data, fg="black")
        
        #Si es gnuradio2BS_DistEst...
        elif topic == list_topic_sub[2]:
            DistEst_label.config(text = rx_data + " m", fg="black")

        #Si es del topico de emergencias
        elif topic == list_topic_sub[3]:
            #Gestionamos las alertas en el dashboard
            handle_emergencies(rx_data)
            """
            #Los mensajes de emergencia vienen como ID;DESCRIPCION
            data_emergencia = rx_data.split(";")
            #El primer valor es el id y el resto es la descripcion
            id_emergency = data_emergencia[0]
            descr_emergency = data_emergencia[1]"""
            
        #Si el topico es el status del MR
        elif topic == list_topic_sub[4]:
            status_MR_label.config(text = rx_data, fg="black")

        
        print("RX from MR: " + rx_data)
            
    #Nos suscribimos a la lista de topicos
    for topico in list_topic_sub:
      client.subscribe(topico)
      print("Subscribed to topic: " + topico)

    client.on_message = on_message


#Función para establecer el ID del MR que se quiere usar
def set_MR_id():
    global id_MR
    id_MR = txt_MR_id.get()
    label_MR_id.config(text = "You are connected to " + id_MR, fg = "black")
    print("El ID-MR es: " + id_MR)



#Funcion para gestionar el comando desde los botones
def handle_btn(value):
    global cmd, clicked, cmd_stop_run, cmd_video, flag_live, cmd_live

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
        print("Flag live:", flag_live)
    
        #Envio start_record
        if value == lista_cmd_camera[1]:
            flag_live = 0
            #Lo siguiente a enviar seria stop_record
            cmd_video = lista_cmd_camera[2]


        #Envio stop_record
        elif value == lista_cmd_camera[2]:
            flag_live = 0
            #Lo siguiente a enviar seria start_record
            cmd_video = lista_cmd_camera[1]

        elif value == lista_cmd_camera[3]:
            flag_live = 1
            cmd_live = lista_cmd_camera[4]
            status_MR_label.config(text="Streaming...")

        elif value == lista_cmd_camera[4]:
            flag_live = 0
            cmd_live = lista_cmd_camera[3]

        print("Flag live: ", flag_live )    
           


    #El valor de movement y el flag clicked son el comando
    cmd = id_MR + ";" + value
    clicked = value
    print("\nCOMMAND: " + cmd)


#==================================================================================================
#=============================================== MQTT =============================================
#==================================================================================================


#==================================================================================================
#===============================================GRAPHICAL USER INTERFACE===========================
#==================================================================================================
window = tk.Tk()
#Width x Height
window.geometry("1100x1000")
window.title("UncertainTIC")
window.resizable(False,False)
window.configure(bg="white")


# ============================================BANNERS======================================================================
#Banner del TITULO
banner_title = tk.Label(text="UncertainTIC PICAR",compound="c",fg="white",bg="black",font=("Helvetica", 12))
#Añado la label a la ventana (si no hago el pack() no se inserta en ella)
banner_title.place(x = 50, y = 10, width=400, height=50)

#Banner de VELOCIDADES
banner_speed = tk.Label(text="Speed Selector",compound="c",fg="white",bg="black",font=("Helvetica", 12))
banner_speed.place(x = 175, y = 380, width=150, height=30)

#Banner de ROTACION
banner_speed = tk.Label(text="Rotation (Z axis)",compound="c",fg="white",bg="black",font=("Helvetica", 12))
banner_speed.place(x = 175, y = 480, width=150, height=30)

#Banner de GNU Radio
banner_speed = tk.Label(text="GNU Radio",compound="c",fg="white",bg="black",font=("Helvetica", 12))
banner_speed.place(x = 175, y = 580, width=150, height=30)

#Banner de CAMARA
banner_speed = tk.Label(text="Camera",compound="c",fg="white",bg="black",font=("Helvetica", 12))
banner_speed.place(x = 175, y = 680, width=150, height=30)

# Banner del SENSING
banner_sensing = tk.Label(text="Received Data from sensors",compound="c",fg="white",bg="black",font=("Helvetica", 12))
banner_sensing.place(x = 550, y = 10 , width=500, height=50)

# Banner de EMERGENCIAS
banner_emergencias = tk.Label(text="Emergency Alarm",compound="c",fg="white",bg="black",font=("Helvetica", 12))
banner_emergencias.place(x = 550, y = 380 , width=500, height=50)

# Banner de STATUS DEL MR
banner_status_MR = tk.Label(text="Mobile Robot Status",compound="c",fg="white",bg="black",font=("Helvetica", 12))
banner_status_MR.place(x = 550, y = 630 , width=500, height=50)
# ============================================BANNERS======================================================================


# ======================================BOTONES DE DIRECCION ===============================================================

#Creo el objeto imagen para el boton de forward
arrow_forward_img = tk.PhotoImage(file = r"./Assets/arrow_forward.png")
#Lo redimensiono
arrow_forward_img = arrow_forward_img.subsample(8, 8)
#Boton forward
btn_forward = tk.Button(window,bg="grey",fg="black", command=lambda:handle_btn(lista_cmd_motores[1]), image = arrow_forward_img)
btn_forward.image = arrow_forward_img
btn_forward.place(x = 225, y = 80, width=50, height=100)

#Creo el objeto imagen para el boton de left
arrow_left_img = tk.PhotoImage(file = r"./Assets/arrow_left.png")
#Lo redimensiono
arrow_left_img = arrow_left_img.subsample(8, 8)
#Boton left
btn_left = tk.Button(window,bg="grey",fg="black",command=lambda:handle_btn(lista_cmd_motores[5]), image = arrow_left_img)
btn_left.image = arrow_left_img
btn_left.place(x = 125, y = 180, width=100, height=50)

#Creo el objeto imagen para el boton de right
arrow_right_img = tk.PhotoImage(file = r"./Assets/arrow.png")
#Lo redimensiono
arrow_right_img = arrow_right_img.subsample(8, 8)
#Boton right
btn_right = tk.Button(window,bg="grey",fg="black",command=lambda:handle_btn(lista_cmd_motores[4]), image = arrow_right_img)
btn_right.image = arrow_right_img
btn_right.place(x =275, y = 180, width=100, height=50)

#Creo el objeto imagen para el boton de left
arrow_back_img = tk.PhotoImage(file = r"./Assets/arrow_back.png")
#Lo redimensiono
arrow_back_img = arrow_back_img.subsample(8, 8)
#Boton backward
btn_backward = tk.Button(window,bg="grey",fg="black",command=lambda:handle_btn(lista_cmd_motores[2]), image = arrow_back_img)
btn_backward.image = arrow_back_img
btn_backward.place(x = 225, y = 230, width=50, height=100)

#Boton stop
btn_stop = tk.Button(window,text="RUN",bg="grey",fg="black", command=lambda:handle_btn(cmd_stop_run), font=("Helvetica", 12))
btn_stop.place(x = 225, y = 180, width=50, height=50)

# ======================================BOTONES DE DIRECCION ===============================================================


# ======================================BOTONES DE VELOCIDAD =================
#Boton velocidad LOW
btn_low = tk.Button(window,text="LOW",bg="grey",fg="black",command=lambda:handle_btn(lista_cmd_motores[8]), font=("Helvetica", 12))
btn_low.place(x = 100, y = 420, width=100, height=30)
#Boton velocidad MEDIUM
btn_med = tk.Button(window,text="MEDIUM",bg="grey",fg="black",command=lambda:handle_btn(lista_cmd_motores[9]), font=("Helvetica", 12))
btn_med.place(x = 200, y = 420, width=100, height=30)
#Boton velocidad HIGH
btn_high = tk.Button(window,text="HIGH",bg="grey",fg="black",command=lambda:handle_btn(lista_cmd_motores[10]), font=("Helvetica", 12))
btn_high.place(x = 300, y = 420, width=100, height=30)
# ======================================BOTONES DE VELOCIDAD =================


# ======================================BOTONES DE ROTACION AXIAL=================
#Boton left EJE
btn_rot_left = tk.Button(window,text="LEFT",bg="grey",fg="black",command=lambda:handle_btn(lista_cmd_motores[7]),font=("Helvetica", 12))
btn_rot_left.place(x = 150, y = 520, width=100, height=30)
#Boton right EJE
btn_rot_right = tk.Button(window,text="RIGHT",bg="grey",fg="black",command=lambda:handle_btn(lista_cmd_motores[6]),font=("Helvetica", 12))
btn_rot_right.place(x = 250, y = 520, width=100, height=30) 
# ======================================BOTONES DE GIRO (AXIAL) =================


# ======================================BOTONES DE GNURADIO =================

#Boton DISTANCIA ESTIMADA DE ANTENA IZQUIERDA
btn_distest_left = tk.Button(window,text="Dist Est Left",bg="grey",fg="black", command=lambda:handle_btn(lista_cmd_gnuradio[0]),font=("Helvetica", 12))
btn_distest_left.place(x = 70, y = 620, width=120, height=30)

#Boton DIRECTION OF ARRIVAL
btn_doa = tk.Button(window,text="DoA + Dist Est",bg="grey",fg="black", command=lambda:handle_btn(lista_cmd_gnuradio[1]),font=("Helvetica", 12))
btn_doa.place(x = 190, y = 620, width=120, height=30)

#Boton DISTANCIA ESTIMADA DE ANTENA DERECHA
btn_distest = tk.Button(window,text="Dist Est Right",bg="grey",fg="black", command=lambda:handle_btn(lista_cmd_gnuradio[2]),font=("Helvetica", 12))
btn_distest.place(x = 310, y = 620, width=120, height=30)

# ======================================BOTONES DE GNURADIO =================


# ======================================BOTON CAMARA============================
#Creo el objeto imagen para el boton de camara
camera_img = tk.PhotoImage(file = r"./Assets/camera.png")
#Lo redimensiono
camera_img = camera_img.subsample(16, 16)
#Boton left EJE (en el parametro image llamo a la foto anterior)
btn_camera = tk.Button(window,bg="grey",fg="black",command=lambda:handle_btn(lista_cmd_camera[0]), image = camera_img)
btn_camera.image = camera_img
btn_camera.place(x = 130, y = 720, width=80, height=40)
# ======================================BOTON CAMARA============================


# ======================================BOTON RECORDING============================
#Creo el objeto imagen para el boton de recording
recording_img = tk.PhotoImage(file = r"./Assets/recording.png")
#Lo redimensiono
recording_img = recording_img.subsample(6, 6)
#Boton left EJE (en el parametro image llamo a la foto anterior)
btn_recording = tk.Button(window,bg="grey",fg="black",command=lambda:handle_btn(cmd_video), image = recording_img)
btn_recording.image = recording_img
btn_recording.place(x = 210, y = 720, width=80, height=40)

# ======================================BOTON LIVE============================
#Creo el objeto imagen para el boton de recording
live_video_img = tk.PhotoImage(file = r"./Assets/live.png")
#Lo redimensiono
live_video_img = live_video_img.subsample(8, 8)
#Boton left EJE (en el parametro image llamo a la foto anterior)
btn_live_video = tk.Button(window,bg="grey",fg="black",command=lambda:handle_btn(cmd_live), image = live_video_img)
btn_live_video.image = live_video_img
btn_live_video.place(x = 290, y = 720, width=80, height=40)


# ====================================== DASHBOARD===========================
#TITULO del label de TEMPERATURA CPU
title_cpu_temp = Label(window,text="MR's CPU Temp:",anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_cpu_temp.place(x=550,y=80, width= 130, height=30)
#LABEL de TEMPERATURA CPU
cpu_temp_label = Label(window,text="",bg="white", anchor = "w", fg="black", font=("Helvetica", 11, 'bold'))
cpu_temp_label.place(x=680,y=80, width= 60, height=30)

#TITULO del label de TEMPERATURA EXTERNA
title_ext_temp = Label(window,text="External Temp:",anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_ext_temp.place(x=760,y=80, width= 120, height=30)
#LABEL de TEMPERATURA EXTERNA
ext_temp_label = Label(window,text="",anchor = "w", bg="white",fg="black", font=("Helvetica", 11, 'bold'))
ext_temp_label.place(x=880,y=80, width= 50, height=30)


#TITULO de MENSAJE DoA desde GNURADIO en el MR
title_DoA = Label(window,text="DoA Debug:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_DoA.place(x=550,y=120, width= 100, height=30)
#LABEL de MENSAJE DoA desde GNURADIO en el MR 
DoA_label = Label(window,text="",anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
DoA_label.place(x=650,y=120, width= 400, height=30)

#TITULO del label de  DISTANCIA ESTIMADA
title_DistEst = Label(window,text="DistEst:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_DistEst.place(x=550,y=160, width= 70, height=30)
#LABEL de MENSAJE DISTANCE ESTIMATION desde GNURADIO en el MR 
DistEst_label = Label(window,text="",anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
DistEst_label.place(x=620,y=160, width= 400, height=30)

#TITULO del label de la DISTANCIA TRASERA (REAR DISTANCE)
title_rear_dist = Label(window,text="Distance Rear Sensor:", anchor ="w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_rear_dist.place(x=550,y=200, width= 170, height=30)
#LABEL de la DISTANCIA del SENSOR 1
dist_rear_label = Label(window,text="", anchor="w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
dist_rear_label.place(x=720,y=200, width= 80, height=30)

#TITULO del label de COLISION TRASERA (REAR DISTANCE COLLISION)
title_col_rear = Label(window,text="COLLISION:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_col_rear.place(x=800,y=200, width= 100, height=30)
#LABEL para mostrar la COLISION TRASERA
coli_rear_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
coli_rear_label.place(x=900,y=200, width= 35, height=30)

#TITULO del label de la DISTANCIA FRONTAL (FRONTAL DISTANCE COLLISION)
title_dist_front = Label(window,text="Distance Front Sensor:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_dist_front.place(x=550,y=240, width= 200, height=30)
#LABEL de la DISTANCIA del SENSOR 2
dist_front_label = Label(window,text="",bg="white", anchor = "w", fg="black",font=("Helvetica", 11, 'bold'))
dist_front_label.place(x=720,y=240, width= 50, height=30)

#TITULO del label de COLISION FRONTAL
title_col_front = Label(window,text="COLLISION:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_col_front.place(x=800,y=240, width= 100, height=30)
#LABEL para mostrar la COLISION 2
coli_front_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
coli_front_label.place(x=900,y=240, width= 30, height=30)


#TITULO del label GIROSCOPIO X
title_giroX = Label(window,text="GYRO X:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_giroX.place(x=550,y=280, width= 80, height=30)
#LABEL para mostrar GIROSCOPIO X
giroX_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
giroX_label.place(x=630,y=280, width= 50, height=30)

#TITULO del label GIROSCOPIO Y
title_giroY = Label(window,text="GYRO Y:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_giroY.place(x=700,y=280, width= 80, height=30)
#LABEL para mostrar GIROSCOPIO Y
giroY_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
giroY_label.place(x=780,y=280, width= 50, height=30)

#TITULO del label GIROSCOPIO Z
title_giroZ = Label(window,text="GYRO Z:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_giroZ.place(x=850,y=280, width= 80, height=30)
#LABEL para mostrar GIROSCOPIO Z
giroZ_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
giroZ_label.place(x=930,y=280, width= 50, height=30)


#TITULO del label ACELEROMETRO X
title_accX = Label(window,text="ACC X:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_accX.place(x=550,y=310, width= 60, height=30)
#LABEL para mostrar ACELEROMETRO X
accX_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
accX_label.place(x=610,y=310, width= 70, height=30)

#TITULO del label ACELEROMETRO Y
title_accY = Label(window,text="ACC Y:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_accY.place(x=680,y=310, width= 60, height=30)
#LABEL para mostrar ACELEROMETRO Y
accY_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
accY_label.place(x=740,y=310, width= 70, height=30)

#TITULO del label ACELEROMETRO Z
title_accZ = Label(window,text="ACC Z:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_accZ.place(x=820,y=310, width= 60, height=30)
#LABEL para mostrar ACELEROMETRO Z
accZ_label = Label(window,text="",anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
accZ_label.place(x=880,y=310, width= 80, height=30)

# ====================================== DASHBOARD (DATA HANDLING) =================

# ====================================== EMERGENCIAS ===============================

#TITULO del label de ID EMERGENCIAS 1
title_emergency_id = Label(window,text="ID:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_emergency_id.place(x=550,y=440, width= 20, height=30)
#LABEL para mostrar ID EMERGENCIAS 1
emergency_id_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
emergency_id_label.place(x=570,y=440, width= 50, height=30)

#TITULO del label del CONCEPTO DE LA EMERGENCIA 1
title_emergency_descr = Label(window,text="Descr:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_emergency_descr.place(x=630,y=440, width= 50, height=30)
#LABEL para mostrar ID EMERGENCIAS 1
emergency_descr_label = Label(window,text="", anchor = "w", bg="white",fg="red",font=("Helvetica", 11, 'bold'))
emergency_descr_label.place(x=680,y=440, width= 400, height=30)


#TITULO del label de ID EMERGENCIAS 2
title_emergency2_id = Label(window,text="ID:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_emergency2_id.place(x=550,y=470, width= 20, height=30)
#LABEL para mostrar ID EMERGENCIAS 2
emergency2_id_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
emergency2_id_label.place(x=570,y=470, width= 50, height=30)

#TITULO del label del CONCEPTO DE LA EMERGENCIA 2
title_emergency2_descr = Label(window,text="Descr:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_emergency2_descr.place(x=630,y=470, width= 50, height=30)
#LABEL para mostrar ID EMERGENCIAS 2
emergency2_descr_label = Label(window,text="", anchor = "w", bg="white",fg="red",font=("Helvetica", 11, 'bold'))
emergency2_descr_label.place(x=680,y=470, width= 400, height=30)


#TITULO del label de ID EMERGENCIAS 3
title_emergency3_id = Label(window,text="ID:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_emergency3_id.place(x=550,y=500, width= 20, height=30)
#LABEL para mostrar ID EMERGENCIAS 3
emergency3_id_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
emergency3_id_label.place(x=570,y=500, width= 50, height=30)

#TITULO del label del CONCEPTO DE LA EMERGENCIA 3
title_emergency3_descr = Label(window,text="Descr:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_emergency3_descr.place(x=630,y=500, width= 50, height=30)
#LABEL para mostrar ID EMERGENCIAS 3
emergency3_descr_label = Label(window,text="", anchor = "w", bg="white",fg="red",font=("Helvetica", 11, 'bold'))
emergency3_descr_label.place(x=680,y=500, width= 400, height=30)


#TITULO del label de ID EMERGENCIAS 4
title_emergency4_id = Label(window,text="ID:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_emergency4_id.place(x=550,y=530, width= 20, height=30)
#LABEL para mostrar ID EMERGENCIAS 4
emergency4_id_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
emergency4_id_label.place(x=570,y=530, width= 50, height=30)

#TITULO del label del CONCEPTO DE LA EMERGENCIA 4
title_emergency4_descr = Label(window,text="Descr:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_emergency4_descr.place(x=630,y=530, width= 50, height=30)
#LABEL para mostrar ID EMERGENCIAS 4
emergency4_descr_label = Label(window,text="", anchor = "w", bg="white",fg="red",font=("Helvetica", 11, 'bold'))
emergency4_descr_label.place(x=680,y=530, width= 400, height=30)


#TITULO del label de ID EMERGENCIAS 5
title_emergency5_id = Label(window,text="ID:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_emergency5_id.place(x=550,y=560, width= 20, height=30)
#LABEL para mostrar ID EMERGENCIAS 5
emergency5_id_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
emergency5_id_label.place(x=570,y=560, width= 50, height=30)

#TITULO del label del CONCEPTO DE LA EMERGENCIA 5
title_emergency5_descr = Label(window,text="Descr:", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
title_emergency5_descr.place(x=630,y=560, width= 50, height=30)
#LABEL para mostrar ID EMERGENCIAS 5
emergency5_descr_label = Label(window,text="", anchor = "w", bg="white",fg="red",font=("Helvetica", 11, 'bold'))
emergency5_descr_label.place(x=680,y=560, width= 400, height=30)

# ====================================== EMERGENCIAS ===============================


# ====================================== MOBILE ROBOT STATUS ===============================
#LABEL para mostrar el STATUS DEL MR
status_MR_label = Label(window,text="", anchor = "w", bg="white",fg="black",font=("Helvetica", 11, 'bold'))
status_MR_label.place(x=550,y=720, width= 300, height=50)
# ====================================== MOBILE ROBOT STATUS ===============================

# ====================================== ELEGIR MR ========================================
#Boton para ELEGIR EL MR
btn_MR_id = tk.Button(window,text="Select",bg="grey",fg="black", command=lambda:set_MR_id(),font=("Helvetica", 12))
btn_MR_id.place(x = 620, y = 690, width=50, height=30)

#Entrada de texto para especificar el ID DE MR
txt_MR_id = Entry(window, bg="white",fg="black",font=("Helvetica", 11, 'bold'))
txt_MR_id.place(x=550,y=690, width= 50, height=30)

#Label donde se MUESTRA el ID DEL MR
label_MR_id = Label(window, text="",  anchor = "w", bg="white",fg="red",font=("Helvetica", 11, 'bold'))
label_MR_id.place(x=680,y=690, width= 250, height=30)
# ====================================== ELEGIR MR ========================================



def run():
    #Nos conectamos al broker
    # Call work function
    client = connect_mqtt()
    client.loop_start()
    subscribe(client)
    while True:
        publish(client)
    
   
if __name__ == '__main__':
    #Hacemos un thread para la funcion run y que esté en paralelo con el bucle infinito de la GUI
    t1=Thread(target=run)
    t1.start()
    window.mainloop()
