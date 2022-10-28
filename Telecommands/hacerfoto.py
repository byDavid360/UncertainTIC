from picamera import PiCamera
import time

#Instanciamos la camara
camera = PiCamera()
time.sleep(2)
camera.capture("prueba.jpg")
print("done")
