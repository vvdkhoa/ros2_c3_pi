#pragma once
#ifndef __MQTT_TOOL__
#define __MQTT_TOOL__

#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <string.h>
//#include "paho-mqtt/MQTTClient.h"
//#include "paho-c/include/MQTTClient.h"
#include <MQTTClient.h>
#if !defined(_WIN32)
#include <unistd.h>
#else
#include <windows.h>
#pragma comment(lib, "paho-mqtt3c.lib")
#endif
//
//#define ADDRESS     "192.168.1.121:1883"
//#define CLIENTID    "ExampleClientSub"
//#define TOPIC       "MQTT Examples"
//#define TOPIC       "test"
//#define PAYLOAD     "Hello World!"
//#define QOS         1
//#define TIMEOUT     10000L


class MQTT_tool
{
private:
    //std::string ClientID;
    static volatile MQTTClient_deliveryToken deliveredtoken;
    static MQTTClient_message message_arrvd[64];
    static std::string message_string[64];
    static std::string message_fromTopic[64];
    static int message_arrvd_ptr;
    static bool Dev_Connect;
    //volatile MQTTClient_deliveryToken deliveredtoken;
    //static MQTTClient client;

    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    MQTTClient_message* Get_message;
    int rc;
    std::string Tag_IP_Address;
    std::string ClientID;
    std::string Topic;
    MQTTClient_message Get_message_List[64];

    int LastMessage_Ptr = 0;
    bool was_connected = false;
    const int MQTT_QOS = 1;
    const long int MQTT_TIMEOUT = 10000L;

    
public:
	MQTT_tool() {
        MQTT_tool::Dev_Connect = false;
    };
	~MQTT_tool() {
        if(was_connected) {
            if ((rc = MQTTClient_disconnect(client, 10000)) != MQTTCLIENT_SUCCESS)
            {
                printf("Failed to disconnect, return code %d\n", rc);
                rc = EXIT_FAILURE;
            }

            destroy_exit();

        }

    };

private:
    static void delivered(void* context, MQTTClient_deliveryToken dt)
    {
        //printf("Message with token value %d delivery confirmed\n", dt);
        MQTT_tool::deliveredtoken = dt;
    }
    static int msgarrvd(void* context, char* topicName, int topicLen, MQTTClient_message* message)
    {
        //printf("Message arrived\n");
        //printf("   context: %s\n", context);
        //printf("     topic: %s\n", topicName);
        //printf("   message: %.*s\n", message->payloadlen, (char*)message->payload);
        MQTTClient_message msg_tmp;
        int message_arrvd_ptr_tmp = (message_arrvd_ptr + 1) % 64;    
        message_arrvd_ptr = message_arrvd_ptr_tmp;
        MQTT_tool::message_arrvd[message_arrvd_ptr_tmp] = *message;

        message_string[message_arrvd_ptr].assign(
            (char*)message_arrvd[message_arrvd_ptr].payload,
            (char*)message_arrvd[message_arrvd_ptr].payload + message_arrvd[message_arrvd_ptr].payloadlen);

        std::string str(topicName);
        message_fromTopic[message_arrvd_ptr] = str;

        MQTTClient_freeMessage(&message);
        MQTTClient_free(topicName);
        return 1;
    }

    static void connlost(void* context, char* cause)
    {
        MQTT_tool::Dev_Connect = false;
        printf("\nConnection lost\n");
        printf("     cause: %s\n", cause);
    }

public:
    bool DevicConnect() {
        return MQTT_tool::Dev_Connect;
    }
    void send(std::string topic, std::string msg) {
        const char* PAYLOAD = msg.c_str();
        const char* TOPIC = topic.c_str();

        pubmsg.payload = const_cast<char*>(PAYLOAD);
        pubmsg.payloadlen = (int)strlen(PAYLOAD);
        pubmsg.qos = MQTT_QOS;
        pubmsg.retained = 0;
        MQTT_tool::deliveredtoken = 0;
        if ((rc = MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token)) != MQTTCLIENT_SUCCESS)
        {
            printf("Failed to publish message, return code %d\n", rc);
            rc = EXIT_FAILURE;
        }
        else
        {
            //printf("Waiting for publication of %s\n"
            //    "on topic %s for client with ClientID: %s\n",
            //    PAYLOAD, TOPIC, ClientID.c_str());
            while (MQTT_tool::deliveredtoken != token)
            {
#if defined(_WIN32)
                Sleep(100);
#else
                usleep(MQTT_TIMEOUT);
#endif
            }
        }
    }
    void sub(std::string topic) {
        const char* TOPIC = topic.c_str();
        if ((rc = MQTTClient_subscribe(client, TOPIC, MQTT_QOS)) != MQTTCLIENT_SUCCESS)
        {
            printf("Failed to subscribe, return code %d\n", rc);
            rc = EXIT_FAILURE;
        }
    }
    bool Init(std::string addr , std::string ID) {
        Tag_IP_Address = addr;
        ClientID = ID;
        rc = MQTTClient_create(&client, Tag_IP_Address.c_str(), ClientID.c_str(), MQTTCLIENT_PERSISTENCE_NONE, NULL);

        if (rc != MQTTCLIENT_SUCCESS)
        {
            printf("Failed to create client, return code %d\n", rc);
            rc = EXIT_FAILURE;
            return was_connected;
        }

        if ((rc = MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered)) != MQTTCLIENT_SUCCESS)
        //if ((rc = MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered)) != MQTTCLIENT_SUCCESS)
        {
            printf("Failed to set callbacks, return code %d\n", rc);
            rc = EXIT_FAILURE;
            destroy_exit();
            return was_connected;
        }

        conn_opts.keepAliveInterval = 20;
        conn_opts.cleansession = 1;
        if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
        {
            printf("Failed to connect, return code %d\n", rc);
            rc = EXIT_FAILURE;
            destroy_exit();
            return was_connected;
        }
        was_connected = true;
        MQTT_tool::Dev_Connect = true;
        return was_connected;
    }
    void destroy_exit() {
        MQTTClient_destroy(&client);
    }
    void unsubscribe() {
        if ((rc = MQTTClient_unsubscribe(client, Topic.c_str())) != MQTTCLIENT_SUCCESS)
        {
            printf("Failed to unsubscribe, return code %d\n", rc);
            rc = EXIT_FAILURE;
        }

    }

    bool GetMessage(std::string& mag) {
        bool res = false;
        if (LastMessage_Ptr != message_arrvd_ptr) {

            LastMessage_Ptr = (LastMessage_Ptr + 1) % 64;            
            mag = message_string[message_arrvd_ptr];
            res = true;
        }
        return res;
    }
    bool GetMessage(std::string& mag, std::string& tpoic) {
        bool res = false;
        if (LastMessage_Ptr != message_arrvd_ptr) {

            LastMessage_Ptr = (LastMessage_Ptr + 1) % 64;
            mag = message_string[message_arrvd_ptr];
            tpoic = message_fromTopic[message_arrvd_ptr];
            res = true;
        }
        return res;
    }

	

};



#endif