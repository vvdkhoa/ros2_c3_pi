#include "MQTT_tool.h"
MQTTClient_message MQTT_tool::message_arrvd[64];
std::string MQTT_tool::message_string[64];
std::string MQTT_tool::message_fromTopic[64];

int MQTT_tool::message_arrvd_ptr = 0;
bool MQTT_tool::Dev_Connect;

volatile MQTTClient_deliveryToken MQTT_tool::deliveredtoken;