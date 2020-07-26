#include <WiFiNINA.h>
#include "DHT.h"
#include <MQTT.h>
#include "secrets.h"                            // Secret file containing Wifi/MQTT secrets (ssid,pass,mqtt key, mqtt secret)
////////////////////////////////////////////////Initialization//////////////////////////////////////////
const int MQTTSendTimeoutMS = 60000;            // Timeout between MQTT publishes
const char mqttServer[] = "broker.shiftr.io";   // broker, with shiftr.io it's "broker.shiftr.io"
const char device[] = "ArduinoMKR1010";         // broker device identifier 
const int mqttServerPort = 1883;                // broker mqtt port
int status = WL_IDLE_STATUS;
#define DHTPIN 7                                 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11                            // DHT 11 or 22
DHT dht(DHTPIN, DHTTYPE);
WiFiClient net;                                  //Instances of WiFiClient and a MQTTClient are needed to communicated remotely.
MQTTClient client;  
////////////////////////////////////////////////Initialization////////////////////////////////////////
void setup() {
  digitalWrite(LED_BUILTIN, HIGH);                // Set error led high when not connected to wifi
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  dht.begin();
}
void loop() {
//////////////////////////////////////////////////Loop////////////////////////////////////////////////
  Serial.print(F("Waiting timeout: "));   // Wait a few seconds between measurements.
  Serial.print(MQTTSendTimeoutMS);        // Wait a few seconds between measurements.
  Serial.println(F(" ms."));              // Wait a few seconds between measurements.
  delay(MQTTSendTimeoutMS);               // Wait a few seconds between measurements.
  float h = dht.readHumidity();           // Reading temperature or humidity takes about 250 milliseconds!
  float t = dht.readTemperature();        // Read temperature as Celsius (the default)
  if (isnan(h) || isnan(t)) {             // Check if any reads failed and exit early (to try again).
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  String temp = String(t, 1);             // Debug output to serial
  String humid = String(h, 1);            // Debug output to serial
  Serial.print(F("Humidity: "));          // Debug output to serial
  Serial.print(temp);                     // Debug output to serial
  Serial.print(F("%  Temperature: "));    // Debug output to serial
  Serial.print(humid);                    // Debug output to serial
  Serial.println(F("°C "));               // Debug output to serial
  connectWifi();                          // Check Wifi connection
  connectMQTT();                          // Check MQTT connection
  client.publish("/temperature-reading", temp);    //Send to MQTT
  Serial.print(F("Sent "));
  Serial.print(temp);
  Serial.println(F(" to MQTT broker "));
///////////////////////////////////////////////////Loop////////////////////////////////////////////////
}
void printWifiStatus() {
  digitalWrite(LED_BUILTIN, LOW);         //Set error led low
  Serial.print("SSID: ");                 // print the SSID of the network you're attached to:
  Serial.println(WiFi.SSID());            // print the SSID of the network you're attached to:
  IPAddress ip = WiFi.localIP();          // print your board's IP address:
  Serial.print("IP Address: ");           // print your board's IP address:
  Serial.println(ip);                     // print your board's IP address:
  long rssi = WiFi.RSSI();                // print the received signal strength:
  Serial.print("signal strength (RSSI):");// print the received signal strength:
  Serial.print(rssi);                     // print the received signal strength:
  Serial.println(" dBm");                 // print the received signal strength:
}

void connectMQTT(){
  client.begin(mqttServer, mqttServerPort, net); //Connect MQTT
  while (!client.connected()) {                 //if not connected: try to reconnect
    Serial.println("MQTT not connected: connecting...");
     if (client.connect(device, key, secret)) { //if not connected: try to reconnect
        Serial.println("connected to MQTT");
      } else {
        Serial.print("failed to connect to MQTT ");
        delay(2000);
        }
    }
}

void connectWifi(){                         // attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Wifi not connected: attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(20000);                           // wait 10 seconds for connection:
  }
  Serial.println("Connected to wifi");
  printWifiStatus();
}
