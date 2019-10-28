#include "mbed.h"
#include "WIZnetInterface.h"

Serial pc(PC_10, NC); // serial output on D26 (pin 4 of UEXT)
DigitalIn BTN(PC_9);
DigitalOut LED(PA_5);

void on_control_cmd(const char* actuator_name, const char* control_value)
{
    pc.printf("Received CMD %s %s\r\n", actuator_name, control_value);
    if(strcmp(actuator_name, "led") == 0)
        LED = atoi(control_value);
}

int main() {

    MQTTSocket sock;
    MClient client(sock);

    //声明所有的传感器，每行一个，每个由名字、单位两部分组成，最后一行必须为空指针作为结尾
    const char* sensors[][2] = {
        "test", " ",
        "button", "V",
        NULL, NULL //最后一行以空指针作为结束标记
    };

    //声明所有的执行器，每行一个，每个由名字、参数类型两部分组成，最后一行必须为空指针作为结尾
    const char* actuators[][2] = {
        "led", "int",
        NULL, NULL //最后一行以空指针作为结束标记
    };
    networking_init(sock, client, "192.168.1.99", sensors, actuators, on_control_cmd);

    bool btn = 0;
    while(1){
        bool newBTN = BTN;
        if(newBTN != btn){
            char buf[16];
            int value = (bool)newBTN;

            sprintf(buf, "%d mV", value);
            publish_value(client,"button",buf);

            btn = newBTN;
        }else{
            client.yield(1000);
            publish_value(client,"test","hello world");
        }
    }
}
