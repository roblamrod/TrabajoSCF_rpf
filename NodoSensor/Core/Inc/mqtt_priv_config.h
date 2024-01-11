/* Private functions for coreMQTT */
/* aluque 2022-12-21 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MQTT_PRIV_CONFIG_H
#define __MQTT_PRIV_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define MQTT_BROKER_ENDPOINT "industrial.api.ubidots.com" //"test.mosquitto.org"
#define MQTT_BROKER_ENDPOINT_IP {169, 53, 160, 46} //{91,121,93,94}
#define MQTT_BROKER_PORT (1883)
#define MQTTCLIENT_IDENTIFIER "BRIyOA0hGBcPJBchBiU6Bjc"
#define TOPIC_COUNT (1)
#define API_KEY "BBUS-85ec295c5776e53fbf9089679853d03cc61"
//#define API_KEY "BBUS-a3d04547b22f02995aa51ecc74d6f8742ce" //fer
#define DEFAULT_TOKEN "BBUS-l8Ckkld4Q9jR1xf0SQAu1eZRoVczdX" // USERNAME
//#define DEFAULT_TOKEN "BBUS-bKuXeaWrsoaSJqVJAYaBHYnCPjkTlQ" //fer
#define pcBaseTopic "/v1.6/devices/Monitor_Node" // TEMPERATURA "SistCiberFis_enrique/SCF/Temp"
#define pcTempTopic "/v1.6/devices/Monitor_Node/temperatura" // TEMPERATURA "SistCiberFis_enrique/SCF/Temp"
#define pcAccTopic "/v1.6/devices/Monitor_Node/aceleraciones" // ACELERACIONES //"SistCiberFis_antonio/SCF/LED"
#define pcHumTopic "/v1.6/devices/Monitor_Node/humedad"
#define pcModOpTopic "/v1.6/devices/Monitor_Node/modo_operacion"
#define pcEstadoTopic "/v1.6/devices/Monitor_Node/estado"

// Define strings for these parameters or set them to NULL. Do not use empty string ("")
#define mqttUserName DEFAULT_TOKEN //NULL
#define clientID NULL
#define mqttPass NULL

#ifdef __cplusplus
extern "C" {
#endif

#endif /* __MQTT_PRIV_CONFIG_H */
