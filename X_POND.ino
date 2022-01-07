// include library
//#include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"
DFRobot_ESP_PH ph;
#define ESPADC 4096.0   //the esp Analog Digital Convertion value
#define ESPVOLTAGE 5000 //the esp voltage supply value
#define PH_PIN 35
int led_pin = 4;
int oneWireBus = 27;
float voltage, phValue, temperatureC = 25;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
#define DO_PIN 34
#define VREF 5000    //VREF (mv)
#define ADC_RES 4096 //ADC Resolution

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1600) //mv
#define CAL1_T (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv  
#define CAL2_T (15)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}
// setting wifi
const char* ssid = ""; // isi nama SSID wifinya
const char* password = ""; // isi password wifinya

//Backend credentials
const char* mqtt_server = "mqtt.flexiot.xl.co.id";
const char* clientId="9985435337972242";
String DEVICE_SERIAL = "9985435337972242" ; //update the device serial according to the serial given by the consumer portal 
const char* EVENT_TOPIC = "flexiot_esp32_demo/esp32_devkit/v35/common";
String SUB_TOPIC_STRING = "+/" + DEVICE_SERIAL + "/flexiot_esp32_demo/esp32_devkit/v35/sub";
const char* mqtt_username = "";
const char* mqtt_password = "";
WiFiClient espClient;
PubSubClient client(espClient);
//char msg[300];
char msg1[300];
char msg2[300];
void setup_wifi() {
 // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);
}
void reconnect() {
  // Loop until we're reconnected
     while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
//    String clientId = "ESP8266Client-";
//    clientId += String(random(0xffff), HEX);

    // Attempt to connect
    if (client.connect(clientId,mqtt_username,mqtt_password)) {
      Serial.println("connected");
      //subscribe to the topic
      const char* SUB_TOPIC = SUB_TOPIC_STRING.c_str();
      client.subscribe(SUB_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
        
    }
  }
}
void readSuhu() {
    sensors.begin();
    sensors.requestTemperatures(); 
    float Temperature = sensors.getTempCByIndex(0); 

    String kirim;
    kirim += "{\"eventName\":\"pengukuran\",\"status\":\"<none>\",\"temp\":\"";
    kirim += Temperature;
    kirim += "\",\"v35\":\""+DEVICE_SERIAL+"\"}";
    char*msg = (char*)kirim.c_str();
    Serial.print("mengirim : ");
    Serial.println(kirim);
    publish_message(msg);

}
void read_PH() {
    ph.begin();
    EEPROM.begin(32);
    sensors.begin();
    sensors.requestTemperatures(); 
    float Temperature = sensors.getTempCByIndex(0);
    voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // read the voltage
    phValue = ph.readPH(voltage, Temperature);
    ph.calibration(voltage, Temperature); // calibration process by Serail CMD
    String kirim1;
    kirim1 += "{\"eventName\":\"pengukuran\",\"status\":\"<none>\",\"phval\":\"";
    kirim1 += phValue;
    kirim1 += "\",\"v35\":\""+DEVICE_SERIAL+"\"}";
    char*msg1 = (char*)kirim1.c_str();
    Serial.print("mengirim : ");
    Serial.println(kirim1);
    publish_message1(msg1);
}
void readDO() {  
  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
  Serial.print("Temperaturet:\t" + String(Temperaturet) + "\t");
  Serial.print("ADC RAW:\t" + String(ADC_Raw) + "\t");
  Serial.print("ADC Voltage:\t" + String(ADC_Voltage) + "\t");
  Serial.println("DO:\t" + String(readDO(ADC_Voltage, Temperaturet)) + "\t");
  DO = readDO(ADC_Voltage, Temperaturet);
  delay(1000);
  String kirim2;
  kirim2 += "{\"eventName\":\"pengukuran\",\"status\":\"<none>\",\"do\":\"";
  kirim2 += DO;
  kirim2 += "\",\"v35\":\""+DEVICE_SERIAL+"\"}";
  char*msg2 = (char*)kirim2.c_str();
  Serial.print("mengirim : ");
  Serial.println(kirim2);
  publish_message2(msg2);
  delay(1000); 
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1893);
  //client.setCallback(callback);
}

void loop() {

  if(WiFi.status() != WL_CONNECTED){
    setup_wifi();  
    //pinMode(led_pin, OUTPUT);
    //digitalWrite(led_pin, HIGH);
  }
  if (WiFi.status()== WL_CONNECTED && !client.connected()) {
    //pinMode(led_pin, OUTPUT);
    //digitalWrite(led_pin, LOW);
    reconnect();  
  }
//  if(WiFi.status() != WL_CONNECTED){
//    pinMode(led_pin, OUTPUT);
//    digitalWrite(led_pin, HIGH);
//  }
//  else{
//    pinMode(led_pin, OUTPUT);
//    digitalWrite(led_pin, LOW);
//  }
  client.loop();
  readSuhu();
  readDO();
  read_PH();
}
void publish_message(const char* msg){
    client.publish(EVENT_TOPIC,msg);
    delay(2000);  
}
void publish_message1(const char* msg1){
  client.publish(EVENT_TOPIC,msg1);
  delay(2000);  
}
void publish_message2(const char* msg2){
  client.publish(EVENT_TOPIC,msg2);
  delay (2000);  
}
