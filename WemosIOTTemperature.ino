    #include <ESP8266WiFi.h>
    #include "DHT.h"
    #include "secrets.h"
    #include <PubSubClient.h>

    const char mqttServer[] = "broker.shiftr.io";   // broker, with shiftr.io it's "broker.shiftr.io"
    const char device[] = "WemosD1Mini";            // broker device identifier 
    const int mqttServerPort = 1883;                // broker mqtt port
    const int MQTTSendTimeoutMS = 60000;             // Timeout between MQTT publishes
    int status = WL_IDLE_STATUS;
    #define DHTTYPE DHT11   
    #define dht_dpin D7                             // Digital pin connected to the DHT sensor
    DHT dht(dht_dpin, DHTTYPE);
    WiFiClient espClient;
    PubSubClient client(espClient);  
    
    void setup() { 
      Serial.begin(115200);
      delay(10);
      dht.begin();
      client.setServer(mqttServer, mqttServerPort);    // Configure MQTT connexion
    }
     
    void loop() {
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
      connectWifi();
      connectMQTT();
      client.publish("/Wemostemp",String(temp).c_str(),true);    //Send to MQTT
      Serial.print(F("Sent "));
      Serial.print(temp);
      Serial.println(F(" to MQTT broker "));   
   }
   
    void connectMQTT(){
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
