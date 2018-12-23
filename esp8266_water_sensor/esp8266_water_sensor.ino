
/*
 Basic ESP8266 MQTT example

 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.

 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.

 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"

*/
#define ARDUINO_ESP8266_ESP01
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "adc.h"

#include <SHT1x.h>
#include <ESPBASE.h>
ESPBASE Esp;

#define dataPin 12
#define clockPin 14
SHT1x sht1x(dataPin, clockPin);

#define water_sensor_pin 13
#define WIFI_WAIT_TIME 120

// Update these with values suitable for your network.

/*
const char* ssid = "Ai";
const char* password = "skoro-otpysk2016";
const char* mqtt_server = "m13.cloudmqtt.com";
const char* mqtt_user = "kmivtgle";
const char* mqtt_passwd = "NlcRwq0HtoLe";
//mqtt_port=14419
ESP8266_wtr_sensor1_humiditySHT10

*/
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 0
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int tm=300;
float temp=0;
int gpio2_pin = 2;

/*
void setup_wifi() {

  delay(10);
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
  
  
}
*/

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  int i;
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    Serial.println(clientId.c_str());
    Serial.println(config.mqtt_username.c_str());
    Serial.println(config.mqtt_password.c_str());
    if (client.connect(clientId.c_str(),config.mqtt_username.c_str(),config.mqtt_password.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      for (i=0;i<50;i++)
      {
        //  feed de DOG :)
        customWatchdog = millis();
        server.handleClient();
        delay(100);
      }
      
    }
  }
}

// Функция отправки показаний с термодатчика
void TempSend()
{
  int payload_len=0;
  String payload;
  char payload_chars[10];
  String s;
  sensors.requestTemperatures(); // от датчика получаем значение температуры
  float temp = sensors.getTempCByIndex(0);
  payload=String(temp);
  payload.toCharArray(payload_chars,10);
  payload_len=payload.length();

  s = "ESP8266_"+config.mqtt_prefix+"_temp";
  client.publish(s.c_str(),payload_chars,payload_len); // отправляем в топик для термодатчика значение температуры
  Serial.println("Temp: ");
  Serial.println(s);
  Serial.println(temp);
  Serial.println("");

  delay(10); 
}



void setup() {
  long start=0;
  //Onboard led
  pinMode(gpio2_pin, OUTPUT);
  digitalWrite(gpio2_pin, LOW); 
  //ADC_MODE(ADC_VCC);
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  sensors.begin();
  Esp.initialize();
  
  //setup_wifi();

  Serial.println("MQTT server: ");
  Serial.println(config.mqtt_server);
  client.setServer(config.mqtt_server.c_str(), atoi(config.mqtt_port.c_str()));
  client.setCallback(callback);

  //ds18b20 init
 Serial.println("ds18b20 devices:");
 Serial.println(sensors.getDeviceCount());
  
  //water sensor
 pinMode(water_sensor_pin, INPUT); 

 start=millis();

 while(!Esp.WIFI_connected)
  {
    server.handleClient();
    customWatchdog = millis();
    //Check if we're stuck here for more than WIFI_WAIT_TIME  seconds. If yes - "reboot" (deep sleep for 5 sec).
    if(customWatchdog - start > WIFI_WAIT_TIME*1000)
    {
      Serial.println("Timeout waiting for WiFi. Rebooting");
      delay(500);
      ESP.deepSleep(5e6); // 30e7 is 300 seconds 
    }
  }
 //Init done
 if(Esp.WIFI_connected)
 {
   digitalWrite(gpio2_pin, HIGH);
 }
 
}

void loop() 
{
  
float voltage=0;
float tempC; 
float humidity;
String payload;
int sensor_wet=0;
int ret=0;
String s;

  char payload_chars[10];
  int payload_len=0;
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
 
  long now = millis();
  customWatchdog = millis();
  //if (now - lastMsg > 2000) {
     TempSend();
    //voltage=analogRead(A0);
    voltage=ESP.getVcc();
    lastMsg = now;
    ++value;
    //snprintf (msg, 75, "hello world #%ld", value);
    //Serial.print("Publish message: ");
    //Serial.println(msg);
    //client.publish("outTopic", msg);
    s = "ESP8266_"+config.mqtt_prefix+"_voltage";
    Serial.println("Voltage: ");
    Serial.println(s);
    Serial.println(voltage);
    Serial.println("");

    payload=String(voltage);
    payload.toCharArray(payload_chars,10);
    payload_len=payload.length();
    
    ret=client.publish(s.c_str(),payload_chars,payload_len);
    Serial.print("Ret: ");
    Serial.println(ret);

    //SHT10 
    s = "ESP8266_"+config.mqtt_prefix+"_tempSHT10";
    tempC = sht1x.readTemperatureC();
    humidity = sht1x.readHumidity();
    Serial.println("Temp SHT10: ");
    Serial.println(s);
    Serial.println(tempC);
    Serial.println("");
    payload=String(tempC);
    payload.toCharArray(payload_chars,10);
    payload_len=payload.length();

    client.publish(s.c_str(),payload.c_str());
    //ret=client.publish("wtr_sensor1_tempSHT10",payload.c_str());
    //client.publish(s.c_str(),payload_chars,payload_len);
    //client.publish("wtr_sensor1_tempSHT10",payload_chars,payload_len);
    Serial.print("Ret: ");
    Serial.println(ret);
    //client.loop();

    //customWatchdog = millis();
    
    s = "ESP8266_"+config.mqtt_prefix+"_humiditySHT10";
    Serial.println("Humidity: ");
    Serial.println(s);
    Serial.println(humidity);
    Serial.println("");
    payload=String(humidity);
    payload.toCharArray(payload_chars,10);
    payload_len=payload.length();
    
    ret=client.publish(s.c_str(),payload_chars,payload_len);
    Serial.print("Ret: ");
    Serial.println(ret);
    //customWatchdog = millis();
    
    
    //Read water sensor
    s = "ESP8266_"+config.mqtt_prefix+"_water_sensor";
    sensor_wet=digitalRead(water_sensor_pin);
    Serial.println("Water sensor: ");
    Serial.println(s);
    Serial.println(sensor_wet);
    Serial.println("");
    payload=String(sensor_wet);
    payload.toCharArray(payload_chars,10);
    payload_len=payload.length();
    s = config.mqtt_prefix+"_water_sensor";
    ret=client.publish(s.c_str(),payload_chars,payload_len);
    Serial.print("Ret: ");
    Serial.println(ret);
    //client.loop();
    delay(1000);
    Serial.println("Going into deep sleep for 300 seconds");
    ESP.deepSleep(30e7); // 30e7 is 300 seconds
  //}
}
