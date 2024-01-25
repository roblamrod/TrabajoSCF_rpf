# Processing Service
# Roberto Lama, Fernando Garrucho y Pablo Kratzer

#!/usr/bin/env python
import paho.mqtt.client 
import time
import json
import threading
import math
import numpy as np
############
DEVICE_LABEL = "Monitor_Node"
broker_address="industrial.api.ubidots.com"
topic = "/v1.6/devices/" + DEVICE_LABEL + "/+"
#token = "BBUS-2AVUwvm4bAkHvZPB4ubI00po18uyOa" # Roberto
#token2 = "BBUS-x0ctVBVOqeus6M6HcB73bwe4vBuwqH" # Roberto
token = "BBUS-8WokbMlGFTnwATPyUtMFoccYgkuYNR" # Fer 1er dispositivo
token2 = "BBUS-D15oqRAF11mvJqLWOdzrrskIzMxXzS" #Fer 2o dispositivo
# Crear lista con las variables 
variables = ["temperatura", "acel_x", "acel_y", "acel_z", "humedad"]

# Crear diccionar con el valor de las variables
values = {"modo_operacion":None, "temperatura": 0, "acel_x": 0, "acel_y": 0, "acel_z": 0, "humedad": 0, "variables_inicializadas": False}
# Crear diccionario para publicar los angulos
angulos = {"ang_x": 0, "ang_y": 0, "ang_z": 0}
# Crear listas para controlar la inicializacion de las variables
variables_init = []
aceleracion_init = []
# Crear lista con las variables de aceleracion
aceleracion_list = ["acel_x", "acel_y", "acel_z"]
aceleracion_old = {"acel_x": None, "acel_y": None, "acel_z": None}
# Referencias de angulos
ANG_REF_Y = np.pi/2
ANG_REF_Z = 0
# Humedad umbral
HUMIDITY_THRESHOLD = 60
ANG_THRESHOLD = 5*math.pi/180 # 5 grados
# Angulo crítico 
ANG_CRITICAL = 25*math.pi/180 # 25 grados

mensaje_recibido = False

client = paho.mqtt.client.Client(client_id='P1')

def on_message(client, userdata, message):
    # Variables globales
    global mensaje_recibido
    global values
    global aceleracion_init
    global variables_init
    global aceleracion_old

    mensaje_recibido = True
    #print("message received ",str(message.payload.decode("utf-8")))
    #print("message topic=",message.topic)
    
    variable = message.topic.split('/')[-1]
    # Guardar el valor obtenido en json
    valor_json = json.loads(message.payload.decode("utf-8"))
    values[variable] = float(valor_json["value"])

    if values["variables_inicializadas"] == False:
        if variable not in variables_init:
            variables_init.append(variable)
        if len(variables_init) == len(variables):
            values["variables_inicializadas"] = True

def on_connect(client, userdata, flags, rc):
    print('connected (%s)' % client._client_id)
    client.subscribe(topic)

def on_disconnect(client, userdata,rc=0):
    print("DisConnected result code "+str(rc))
    client.loop_stop()

# Calculamos el ángulo de inclinación del dispositivo con  respecto a la vertical
def ang_inclinacion (ax,ay,az):
    accel_ang_incl_x = math.atan(ay/np.sqrt(math.pow(ax,2) + math.pow(az,2)))
    accel_ang_incl_y = math.atan(az/np.sqrt(math.pow(ax,2) + math.pow(ay,2)))
    accel_ang_incl_z = math.atan(ax/np.sqrt(math.pow(ay,2) + math.pow(az,2)))
    return accel_ang_incl_x, accel_ang_incl_y, accel_ang_incl_z
 


client.on_connect = on_connect
#client.on_subscribe = on_suscribe
client.on_message = on_message
client.on_disconnect = on_disconnect
client.username_pw_set(token, "")


client.connect(host=broker_address, port=1883)
# Crea un proceso llamando a client.loop_forever() utilizando el modulo threading

# Crear un diccionario compartido entre los procesos para guardar los valores de las variables

p1 = threading.Thread(target=client.loop_forever)
p1.start()


def main():
    # Variables globales
    global mensaje_recibido
    global values

    diccionario_publicacion = {}
    diccionario_publicacion_2 = {}
    angulos_old = {"ang_x": None, "ang_y": None, "ang_z": None}
    humidity_threshold_flag=False
    ang_threshold_flag=False
    ang_critical_flag=False
    old_critical =  0


    while not values["variables_inicializadas"]:
        time.sleep(0.1)
        print("Esperando a que se inicialicen las variables...")
    print("Variables inicializadas")

    while(True): # Bucle infinito
        if values["variables_inicializadas"]==True and mensaje_recibido==True:
            print(values)
            diccionario_publicacion["ang_x"], diccionario_publicacion["ang_y"], diccionario_publicacion["ang_z"] = ang_inclinacion(values["acel_x"], values["acel_y"], values["acel_z"])
            aceleracion_init = []
            mensaje_recibido = False
            values["variables_inicializadas"]==False

            # Si alguno de los angulos supera un umbral y el modo de operacion es 0 (Normal) se cambia a modo 1 (Alarma)
            # Si los valores de aceleracion_old no son None
            print("Diferencia eje y: ", np.abs(ANG_REF_Y-diccionario_publicacion["ang_y"]), " Diferencia eje z: ", np.abs(ANG_REF_Z-diccionario_publicacion["ang_z"]), "Threshold: ", ANG_THRESHOLD)

            if np.abs(ANG_REF_Y-diccionario_publicacion["ang_y"]) > ANG_THRESHOLD or np.abs(ANG_REF_Z-diccionario_publicacion["ang_z"]) > ANG_THRESHOLD and values["modo_operacion"] == 0: # angulos["ang_x"] > ANG_THRESHOLD or 
                print(">>>> Angulo alto")
                ang_threshold_flag=True
                if np.abs(ANG_REF_Y-diccionario_publicacion["ang_y"]) > ANG_CRITICAL or np.abs(ANG_REF_Z-diccionario_publicacion["ang_z"]) > ANG_CRITICAL:
                    print(">>>> Angulo critico")
                    ang_critical_flag=True
                    diccionario_publicacion_2["estado"] = 1
            else:
                print(">>>> Angulo bajo")
                ang_threshold_flag=False
                ang_critical_flag=False
                diccionario_publicacion_2["estado"] = 0

        # Si la humedad supera un umbral y el modo de operacion es 0 (Normal) se cambia a modo 1 (Alarma)
        if values["humedad"] > HUMIDITY_THRESHOLD and values["modo_operacion"] == 0:
            print(">>>> Humedad alta")
            humidity_threshold_flag=True
        elif values["humedad"] <= HUMIDITY_THRESHOLD and values["modo_operacion"] == 1:
            print(">>>> Humedad baja")
            humidity_threshold_flag=False
        else:
            print(">>>> Modo alarma y humedad alta -> humedad: ", values["humedad"], " modo_operacion: ", values["modo_operacion"])
    
        # Si se cumple alguna de las condiciones anteriores se cambia el modo de operacion
        if humidity_threshold_flag or ang_threshold_flag:
            print(">>>> Cambio de modo de operacion a modo 1")
            diccionario_publicacion["modo_operacion"]= 1 # Modo operacion 1: Humedad alta (Alarma)
            client.publish("/v1.6/devices/" + DEVICE_LABEL, json.dumps(diccionario_publicacion))
        elif not humidity_threshold_flag and not ang_threshold_flag:
            diccionario_publicacion["modo_operacion"]= 0 # Modo operacion 0: Normal
            print(">>>> Cambio de modo de operacion a modo 0")
            client.publish("/v1.6/devices/" + DEVICE_LABEL, json.dumps(diccionario_publicacion))
        
        print("Estado critico", diccionario_publicacion_2["estado"])
        #print("Estado old_critical", old_critical)
        #if old_critical != diccionario_publicacion_2["estado"]: # ang_critical_flag:
        #   print(">>>> Angulo critico")
        client.publish("/v1.6/devices/" + DEVICE_LABEL, json.dumps(diccionario_publicacion_2))
        #old_critical = diccionario_publicacion_2["estado"]

        #print("message qos=",message.qos)
        #print("message retain flag=",message.retain)

        time.sleep(5) # Esperar 0.1 segundo
###########################################

main()

p1.join() # Queda esperando a que termine el hilo h2




