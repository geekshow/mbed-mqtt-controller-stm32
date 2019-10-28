/*
  MQTT Sensors by Keith Patterson

  (c) 2018

  v2 - changed to Ethernet_STM driver (from Ethernet_STM32)
  v3 - added timers, processing of incoming relay commands, split out ethernet fn, multiple DHTs, watchdog
  v4 - add inputs, organise pin definitions
  v5 - introduce arrays, tidy DHT code
  2019-03-28: add DS18B20 lib, functions

  TODO
  - tidy up
  - RTOS?
*/

#include <SPI.h>
#include <Ethernet_STM32.h>   // lib for only W5500 LAN module
//#include <Ethernet_STM.h>   // lib for older W5100 LAN module
#include <PubSubClient.h>
#include "DHT.h"
#include <OneWireSTM.h>
#include <libmaple/iwdg.h>    // watchdog lib

// ========== PIN DEFINITIONS ============
#define NUM_OF_OUTPUTS 11
#define NUM_OF_INPUTS 12
#define NUM_OF_DHTS 1
#define NUM_OF_DS 4

#define GREENLEDPIN PA5
#define ORANGELEDPIN PA1

#define ANALOGPIN0 PC0  // Analogue Input 0
#define ANALOGPIN1 PC1  // Analogue Input 1
#define ANALOGPIN2 PC2  // Analogue Input 2
#define ANALOGPIN3 PC3  // Analogue Input 3
#define ANALOGPIN4 PC4  // Analogue Input 4
#define ANALOGPIN5 PC5  // Analogue Input 5

//#define INPUTPINx PC15     // Don't use D23
#define INPUTPIN0 PB9     // DHT22 digital pin (D24)
#define INPUTPIN1 PD2     // DHT22 digital pin (D25)
#define INPUTPIN2 PC10     // DHT22 digital pin (D26)
//#define INPUTPINx PB0     // Don't use D27
//#define INPUTPINx PB1     // Don't use D28
#define INPUTPIN3 PB10     //  digital pin (D29)
#define INPUTPIN4 PB11     //  digital pin (D30)
#define INPUTPIN5 PB12     //  digital pin (D31)
#define INPUTPIN6 PB13     //  digital pin (D32)
#define INPUTPIN7 PB14     //  digital pin (D33)
#define INPUTPIN8 PB15     //  digital pin (D34)
#define INPUTPIN9 PC6      //  digital pin (D35)
#define INPUTPIN10 PC7     //  digital pin (D36)
#define INPUTPIN11 PC8     //  digital pin (D37)

#define OUTPUTPIN0 PA3  // digital output D0
#define OUTPUTPIN1 PA2  // digital output D1
#define OUTPUTPIN2 PA0  // digital output D2
#define OUTPUTPIN3 PA1  // digital output D3
#define OUTPUTPIN4 PB5  // digital output D4
#define OUTPUTPIN5 PB6  // digital output D5
#define OUTPUTPIN6 PA8  // digital output D6
#define OUTPUTPIN7 PA9  // digital output D7
#define OUTPUTPIN8 PA10  // digital output D8
#define OUTPUTPIN9 PB7  // digital output D9
//#define OUTPUTPINx PA4  // digital output D10 - SPI1 SS
//#define OUTPUTPINx PA4  // digital output D11 - SPI1 MOSI
//#define OUTPUTPINx PA4  // digital output D12 - SPI1 MISO
//#define OUTPUTPINx PA4  // digital output D13 - SPI1 CLK
#define OUTPUTPIN10 PB8  // digital output D14
// ================= *************** ==================


#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define W5100_ETHERNET_SHIELD // Arduino Ethernet Shield and Compatibles
#define W5500_ETHERNET_SHIELD   // WIZ550io, ioShield series of WIZnet
#define STATUS_ANNOUNCE_PERIOD 1000000  // how often to announce uptime
#define MQTT_TOPIC_OUT "controller102"
#define MQTT_TOPIC_IN "controller102/command/#"
#define MQTT_TOPIC_OUTPUT "controller102/command/output"
#define MQTT_PORT  1883
#define MQTT_MSG_LEN  50

String ONOFF[] {"OFF","ON"};
String OPENCLOSE[] {"CLOSED","OPEN"};
String YESNO[] {"NO","YES"};

float h[NUM_OF_DHTS];
float t[NUM_OF_DHTS];
float temp[NUM_OF_DS];
unsigned long uptimeSec;
bool outputs[NUM_OF_OUTPUTS];
bool inputs[NUM_OF_INPUTS];
bool SecTimerFlag;
bool SampleDsFlag;

// Initialize DHT sensor.
DHT dht0(ANALOGPIN0, DHTTYPE);

// Initialize DS18B20 sensor.
OneWire  ds(ANALOGPIN5);

byte mac[]    = {  0x00, 0x00, 0x00, 0xBE, 0xEF, 0x02 };
IPAddress ip(192, 168, 1, 222);       // not used unless DHCP deactivated - remove?
IPAddress server(192, 168, 1, 99);    // MQTT broker


void MQTTcallback(char* topic, byte* payload, unsigned int length) {
  char payloadChar[20];
  memcpy(payloadChar, payload, length);
  payloadChar[length] = '\0';
  int topicLength = strlen(MQTT_TOPIC_OUTPUT);
  unsigned int outputNum;
  // DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.print(" - ");
  Serial.println(payloadChar);
  
  // check topic name matches
  if( strncmp(topic, MQTT_TOPIC_OUTPUT, topicLength) == 0 ) {
    // check output number is in range
    outputNum = int(topic[topicLength])-48;
    if( outputNum >= NUM_OF_OUTPUTS) {
      Serial.println("ERROR: Requested output out of range");
      mqttPublishString("error","Requested output out of range");
    }
    else {
      Serial.print("Changing output: ");
      Serial.print(outputNum);
      Serial.print(" to: ");
      // on or off?
      if (strcmp(payloadChar, "ON") == 0) {
        Serial.println("on");
        outputs[outputNum] = 1;
      }
      else if (strcmp(payloadChar, "OFF") == 0) {
        Serial.println("off");
        outputs[outputNum] = 0;
      }
      else {
        Serial.println("Requested output status not known");
        mqttPublishString("error","Requested output status not known");
      }
    }
  }
}

EthernetClient ethClient;
PubSubClient client(server,MQTT_PORT,MQTTcallback,ethClient);

SPIClass mSPI(1); // use SPI channel 1 (pins D10-D13)

void connectEthernet() {
//  int attempts=0;
  // check ethernet connection first
  Serial.print("Attempting ethernet connection...");
  if (!Ethernet.begin(mac)) {
    Serial.println("Failed to configure Ethernet using DHCP");
    delay(300);
  }
  Serial.print("IP Address: ");
  Serial.println(String(String(Ethernet.localIP()[0]) + "." + String(Ethernet.localIP()[1]) +"." + String(Ethernet.localIP()[2]) + "." + String(Ethernet.localIP()[3])));
}

void connectMQTT() {
  // try MQTT connection...
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(MQTT_TOPIC_OUT)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(MQTT_TOPIC_OUT,"hello world from controller!");
      mqttPublishString("IPAddress",String(String(Ethernet.localIP()[0]) + "." + String(Ethernet.localIP()[1]) +"." + String(Ethernet.localIP()[2]) + "." + String(Ethernet.localIP()[3])));
      
      // ... and resubscribe
      client.subscribe(MQTT_TOPIC_IN);
    } 
    else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
    }
  }
}

void mqttPublishString(char mqttTopic[MQTT_MSG_LEN], String mqttMsgString) {
  // publish strings as character arrays to MQTT server
  char mqttTopicChar[MQTT_MSG_LEN];
  char mqttMsgChar[MQTT_MSG_LEN];
  String mqttTopicString;

  mqttTopicString = String(String(MQTT_TOPIC_OUT) + String("/") + String(mqttTopic));
  mqttTopicString.toCharArray(mqttTopicChar,MQTT_MSG_LEN);
  mqttMsgString.toCharArray(mqttMsgChar,MQTT_MSG_LEN);
  client.publish(mqttTopicChar,mqttMsgChar);
}

void readDHT() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Pick a random sensor at a time, too slow to read all of them in one go!
  long selection = random(NUM_OF_DHTS);
  switch (selection) {
    case 0:
      h[selection] = dht0.readHumidity();
      t[selection] = dht0.readTemperature();
      break;
  }
  Serial.print("Read from DHT number ");
  Serial.println(selection);
}

void sampleDS() {
  // Send sample temperature cmd to DS18B20 sensors
  byte onewire_addr[8];
  byte i;
  
  ds.reset_search();
  // loop through all sensors
  for ( i=0; i<NUM_OF_DS; i++ ) {
    if ( !ds.search(onewire_addr)) {
      Serial.println("DS18B20 Onewire: No more addresses.");
      return;
    }
    if (OneWire::crc8(onewire_addr, 7) != onewire_addr[7]) {
      Serial.println("DS18B20 Onewire: CRC is not valid!");
      return;
    }
    ds.reset();
    ds.select(onewire_addr);
    ds.write(0x44,0);        // start conversion
  }
}

void readDS() {
  // Read temperature from DS18B20 sensors
  byte onewire_addr[8];
  byte data[12];
  byte i,j;
  
  ds.reset_search();
  // loop through all sensors
  for ( i=0; i<NUM_OF_DS; i++ ) {
    if ( !ds.search(onewire_addr)) {
      Serial.println("DS18B20 Onewire: No more addresses.");
      return;
    }
    ds.reset();
    ds.select(onewire_addr);    
    ds.write(0xBE);         // Read Scratchpad
    
    for ( j = 0; j < 9; j++) {           // we need 9 bytes
      data[j] = ds.read();
    }
  
    // Convert the data to actual temperature
    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  
    temp[i] = (float)raw / 16.0;
    Serial.print("DS18B20 #");
    Serial.print(i);
    Serial.print(" Temperature = ");
    Serial.print(temp[i]);
    Serial.println(" C");
  }
}


void readInputs()
{
  inputs[0] = digitalRead(INPUTPIN0);
  inputs[1] = digitalRead(INPUTPIN1);
  inputs[2] = digitalRead(INPUTPIN2);
  inputs[3] = digitalRead(INPUTPIN3);
  inputs[4] = digitalRead(INPUTPIN4);
  inputs[5] = digitalRead(INPUTPIN5);
  inputs[6] = digitalRead(INPUTPIN6);
  inputs[7] = digitalRead(INPUTPIN7);
  inputs[8] = digitalRead(INPUTPIN8);
  inputs[9] = digitalRead(INPUTPIN9);
  inputs[10] = digitalRead(INPUTPIN10);
  inputs[11] = digitalRead(INPUTPIN11);
}

void setOutputs()
{
  digitalWrite(OUTPUTPIN0,!outputs[0]);
  digitalWrite(OUTPUTPIN1,!outputs[1]);
  digitalWrite(OUTPUTPIN2,!outputs[2]);
  digitalWrite(OUTPUTPIN3,!outputs[3]);
  digitalWrite(OUTPUTPIN4,!outputs[4]);
  digitalWrite(OUTPUTPIN5,!outputs[5]);
  digitalWrite(OUTPUTPIN6,!outputs[6]);
  digitalWrite(OUTPUTPIN7,!outputs[7]);
  digitalWrite(OUTPUTPIN8,!outputs[8]);
  digitalWrite(OUTPUTPIN9,!outputs[9]);
  digitalWrite(OUTPUTPIN10,!outputs[10]);
}

void publishStatus() {
  // publish status periodically of sysinfo, inputs and outputs
  mqttPublishString("uptime", String(uptimeSec));
  mqttPublishString("alive", String(ONOFF[1]));
}

void publishInputsOutputs() {
  // announce current status via MQTT
  int i;

  mqttPublishString("status/input0",String(OPENCLOSE[inputs[0]]));
  mqttPublishString("status/input1",String(OPENCLOSE[inputs[1]]));
  mqttPublishString("status/input2",String(OPENCLOSE[inputs[2]]));
  mqttPublishString("status/input3",String(OPENCLOSE[inputs[3]]));
  mqttPublishString("status/input4",String(OPENCLOSE[inputs[4]]));
  mqttPublishString("status/input5",String(OPENCLOSE[inputs[5]]));
  mqttPublishString("status/input6",String(OPENCLOSE[inputs[6]]));
  mqttPublishString("status/input7",String(OPENCLOSE[inputs[7]]));
  mqttPublishString("status/input8",String(OPENCLOSE[inputs[8]]));
  mqttPublishString("status/input9",String(OPENCLOSE[inputs[9]]));
  mqttPublishString("status/input10",String(OPENCLOSE[inputs[10]]));
  mqttPublishString("status/input11",String(OPENCLOSE[inputs[11]]));
  
  mqttPublishString("status/output0",String(ONOFF[outputs[0]]));
  mqttPublishString("status/output1",String(ONOFF[outputs[1]]));
  mqttPublishString("status/output2",String(ONOFF[outputs[2]]));
  mqttPublishString("status/output3",String(ONOFF[outputs[3]]));
  mqttPublishString("status/output4",String(ONOFF[outputs[4]]));
  mqttPublishString("status/output5",String(ONOFF[outputs[5]]));
  mqttPublishString("status/output6",String(ONOFF[outputs[6]]));
  mqttPublishString("status/output7",String(ONOFF[outputs[7]]));
  mqttPublishString("status/output8",String(ONOFF[outputs[8]]));
  mqttPublishString("status/output9",String(ONOFF[outputs[9]]));
  mqttPublishString("status/output10",String(ONOFF[outputs[10]]));
  
  mqttPublishString("status/temperature0",String(t[0]));
  mqttPublishString("status/humidity0", String(h[0]));
  mqttPublishString("status/watertemp1",String(temp[0]));
  mqttPublishString("status/watertemp2",String(temp[1]));
  mqttPublishString("status/watertemp3",String(temp[2]));
  mqttPublishString("status/watertemp4",String(temp[3]));
}

void initEthernet()
{
  // initialise interface and hardware
  Ethernet.init(mSPI, PA4); // SPI object, chip select pin

  // Allow the hardware to sort itself out
  delay(1500);
}

void do_every_1000ms() {
  uptimeSec+=1;
  SecTimerFlag = 1;  // set a flag to process in main loop()
}

/////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);  

  dht0.begin();
  
  // setup output pins
  pinMode(OUTPUTPIN0,OUTPUT);
  pinMode(OUTPUTPIN1,OUTPUT);
  pinMode(OUTPUTPIN2,OUTPUT);
  pinMode(OUTPUTPIN3,OUTPUT);
  pinMode(OUTPUTPIN4,OUTPUT);
  pinMode(OUTPUTPIN5,OUTPUT);
  pinMode(OUTPUTPIN6,OUTPUT);
  pinMode(OUTPUTPIN7,OUTPUT);
  pinMode(OUTPUTPIN8,OUTPUT);
  pinMode(OUTPUTPIN9,OUTPUT);
  pinMode(OUTPUTPIN10,OUTPUT);

  // setup input pins
  pinMode(INPUTPIN0,INPUT_PULLUP);
  pinMode(INPUTPIN1,INPUT_PULLUP);
  pinMode(INPUTPIN2,INPUT_PULLUP);
  pinMode(INPUTPIN3,INPUT_PULLUP);
  pinMode(INPUTPIN4,INPUT_PULLUP);
  pinMode(INPUTPIN5,INPUT_PULLUP);
  pinMode(INPUTPIN6,INPUT_PULLUP);
  pinMode(INPUTPIN7,INPUT_PULLUP);
  pinMode(INPUTPIN8,INPUT_PULLUP);
  pinMode(INPUTPIN9,INPUT_PULLUP);
  pinMode(INPUTPIN10,INPUT_PULLUP);
  pinMode(INPUTPIN11,INPUT_PULLUP);

  // setup timers
  Timer2.pause();
  Timer2.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  Timer2.setCount(0);
  Timer2.setPeriod(STATUS_ANNOUNCE_PERIOD);
  Timer2.setCompare(TIMER_CH1, 1);
  Timer2.attachInterrupt(TIMER_CH1, do_every_1000ms);
  Timer2.resume();

  initEthernet();
  interrupts();

  // init watchdog
  iwdg_init(IWDG_PRE_256, 7*156); // watchdog will reboot if no pulses after 7 seconds
}

void loop()
{
  // sort out network and MQTT connection
  if (!client.connected()) {
    connectEthernet();
    connectMQTT();
  }
  client.loop();

  setOutputs();
  readInputs();

  if( SecTimerFlag ) {   // triggered by periodic HW timer
    publishStatus();
    publishInputsOutputs();
    if(uptimeSec%3 == 0) {
      readDHT();
      readDS();
      sampleDS();
      SampleDsFlag = 1;
    }
    else if (SampleDsFlag == 1) {
    }
    iwdg_feed();     // kick watchdog to avoid a reboot
    SecTimerFlag = 0;
  }
}
