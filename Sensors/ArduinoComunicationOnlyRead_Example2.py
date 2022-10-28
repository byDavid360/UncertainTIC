#!/usr/bin/env python3


import serial

#Abrimos el fichero
fichero = open("data_sensor_rx.txt", "w")


if __name__ == '__main__':
        #Importante usar los mismos baudios que en Arduino
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        #Limpiar datos que no queremos al inicio de la comunicacion
        ser.reset_input_buffer()
        lineTotal =" "
        while True:
                #Si se RX datos...
                if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8').rstrip()
                        lineTotal = lineTotal + line + "\n"
                        print(lineTotal)
                        #Escribimos los datos recibidos en el fichero
                        fichero.write(line)
                        fichero.write("\n")

        #Cerramos la escritura de los datos en el fichero
        fichero.close()


