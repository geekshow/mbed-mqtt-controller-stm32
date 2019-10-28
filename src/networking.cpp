#include "mbed.h"
#include "WIZnetInterface.h"
#include "MQTTSocket.h"
#include "MQTTClient.h"
#include "networking.h"

extern Serial pc;
 //W5500 pinout mosi,miso,sclk,cs,reset
WIZnetInterface wiz(PA_7,PA_6,PA_5,PA_4,PB_10);

#define NODE_NAME "controller03"

uint8_t mac_addr[6]={0x00, 0x00, 0x00, 0xBE, 0xEF, 0x03};

recv_ctl_cb g_recv_cb;
const char* (*g_actuators)[2];
static Timer g_timer;


void meta_report(MClient& client, const char* ns, const char* type,
                    const char* payload = NULL, size_t payload_len = 0,
                    bool retain = false, MQTT::QoS qos = MQTT::QOS1){
    char topic[64];
    sprintf(topic, "/%s/" NODE_NAME "/%s", ns, type);
    int ret = client.publish(topic, (void*)payload, payload_len, qos, retain);
    //pc.printf("client.publish()=%d\r\n",ret);
}


void messageArrived(MQTT::MessageData& md)
{
    MQTT::Message &message = md.message;
//    pc.printf("messageArrived '%s' %d,%d\r\n", md.topicName.cstring, md.topicName.lenstring.len, message.payloadlen);

//    char buf[64];
//    int value, len = sizeof(buf)-1;
//    if(message.payloadlen < len)
//        len = message.payloadlen;
//    memcpy(buf, message.payload, len);
//    buf[len] = '\0';
//    sscanf(buf, "%d", &value);
//    pc.printf("received %d\r\n", value);

    char* payload = new char[message.payloadlen+1];
    if(!payload)
        return;
    memcpy(payload, message.payload, message.payloadlen);
    payload[message.payloadlen]='\0';

    char* topic = new char[md.topicName.lenstring.len+1];
    if(!topic){
        delete[] payload;
        return;
    }
    memcpy(topic, md.topicName.lenstring.data, md.topicName.lenstring.len);
    topic[md.topicName.lenstring.len]='\0';

    char *pch = strtok (topic,"/");
    for (int tok=0; tok<2 && pch != NULL; tok++)
    {
//        pc.printf ("%s\r\n",pch);
        pch = strtok (NULL, "/");
    }
    if(pch)
        g_recv_cb(pch, payload);
    delete[] topic;
    delete[] payload;
}

void publish_value(MClient &client, const char *topic, const char *buf)
{
    meta_report(client, "values",topic,buf,strlen(buf),true);
}

void buildCapability(char *out, const char* infoList[][2])
{
    out[0] = '\0';
    for (int i = 0; infoList[i][0]; ++i)
    {
        strcat(out, infoList[i][0]);
        strcat(out, ",");
        strcat(out, infoList[i][1]);
        strcat(out, "\\n");
    }
}

int networking_init(MQTTSocket &sock, MClient &client, const char *broker,const char* sensors[][2], const char* actuators[][2], recv_ctl_cb cb) {
    int ret;
    g_timer.start();
    disable_LSE(); //free LSE pins
    wiz.init(mac_addr);
    pc.printf("DHCP...\r\n");
    wiz.connect();
    pc.printf("IP: %s\r\n", wiz.getIPAddress());

    srand(rand()^g_timer.read_us());

    ret = sock.connect((char*)broker,1883);
    if(ret != 0){
        pc.printf("failed to connect to TCP server\r\n");
        return 1;
    }
    pc.printf("sock.connect()=%d\r\n",ret);

    srand(rand()^g_timer.read_us());

    ret = client.connect();
    if(ret != 0){
        pc.printf("MQTT connect failed\r\n");
        return 1;
    }
    pc.printf("client.connect()=%d\r\n",ret);


    ret = client.subscribe("/control/" NODE_NAME "/+", MQTT::QOS1, messageArrived);
    pc.printf("sock.subscribe()=%d\r\n", ret);

    g_recv_cb = cb;
    g_actuators = actuators;

    char * capabilities = new char[128];
    if(!capabilities){
        pc.printf("failed to alloc memory\r\n");
        return 1;
    }
    //for (int i = 0; actuators[i][0]; ++i){
//        sprintf(capabilities,"/control/" NODE_NAME "/+",actuators[i][0]);
//        ret = client.subscribe(capabilities, MQTT::QOS1, messageArrived);
//        pc.printf("sock.subscribe(%s)=%d\r\n", capabilities, ret);
//    }

    //节点上线消息
    meta_report(client, "events","online");

    //报告所有可接受的控制指令
    buildCapability(capabilities, actuators);
    meta_report(client, "capability","control", capabilities, strlen(capabilities), true);
    //报告所有的传感器
    buildCapability(capabilities, sensors);
    meta_report(client, "capability","values", capabilities, strlen(capabilities), true);
    delete[] capabilities;

    pc.printf("Initialization done.\r\n");

    return 0;
}
