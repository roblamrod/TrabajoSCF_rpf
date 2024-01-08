# Processing Service
# Roberto Lama, Fernando Garrucho y Pablo Kratzer

#!/usr/bin/env python
import paho.mqtt.client 
import time
import json
import threading

############
DEVICE_LABEL = "Monitor_Node"
broker_address="industrial.api.ubidots.com"
topic = "/v1.6/devices/" + DEVICE_LABEL + "/+"
#token = "BBUS-2AVUwvm4bAkHvZPB4ubI00po18uyOa" # Roberto
token = "BBUS-8WokbMlGFTnwATPyUtMFoccYgkuYNR" # Fer
# Crear lista con las variables 
variables = ["temperatura", "acel_x", "acel_y", "acel_z", "humedad"]

# Crear diccionar con el valor de las variables
values = {"temperatura": 0, "acel_x": 0, "acel_y": 0, "acel_z": 0, "humedad": 0, "variables_inicializadas": False}
variables_init = []
mensaje_recibido = False

def on_message(client, userdata, message):
    # Variables globales
    global mensaje_recibido
    global values
    
    mensaje_recibido = True
    print("message received ",str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    variable = message.topic.split('/')[-1]
    # Guardar el valor obtenido en json
    valor_json = json.loads(message.payload.decode("utf-8"))
    values[variable] = float(valor_json["value"])

    if values["variables_inicializadas"] == False:
        if variable not in variables_init:
            variables_init.append(variable)
        if len(variables_init) == len(variables):
            values["variables_inicializadas"] = True
    
    #print("message qos=",message.qos)
    #print("message retain flag=",message.retain)

def on_connect(client, userdata, flags, rc):
    print('connected (%s)' % client._client_id)
    client.subscribe(topic)

def on_disconnect(client, userdata,rc=0):
    print("DisConnected result code "+str(rc))
    client.loop_stop()

def main():
    # Variables globales
    global mensaje_recibido
    global values

    while not values["variables_inicializadas"]:
        time.sleep(0.1)
        print("Esperando a que se inicialicen las variables...")
    print("Variables inicializadas")

    while(True): # Bucle infinito
        if mensaje_recibido:
            mensaje_recibido = False
            print(values)
        time.sleep(0.1) # Esperar 1 segundo
########################################
 
client = paho.mqtt.client.Client(client_id='P1')

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

main()


p1.join() # Queda esperando a que termine el hilo h2




