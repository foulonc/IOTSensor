    #include <ESP8266WiFi.h>
    #include "secrets.h"
    #include <PubSubClient.h>
    //#include <Wire.h>
    //#include <ArduinoOTA.h>
    #include <farmerkeith_BMP280.h>

    
    const bool bme280Debug = 0;                     // set to 1 to enable printing of BME280 or BMP280 transactions
    bme280 bme0 (0, bme280Debug) ; // creates object bme0 of type bme280, base address

    // BME280 configuration settings
    const byte osrs_t = 2; // setting for temperature oversampling
    // No. of samples = 2 ^ (osrs_t-1) or 0 for osrs_t==0
    const byte osrs_p = 5; // setting for pressure oversampling
    // No. of samples = 2 ^ (osrs_p-1) or 0 for osrs_p==0
    const byte osrs_h = 5; // setting for humidity oversampling
    // No. of samples = 2 ^ (osrs_h-1) or 0 for osrs_h==0
    
    const char mqttServer[] = "broker.shiftr.io";   // broker, with shiftr.io it's "broker.shiftr.io"
    const char device[] = "WemosD1Mini";            // broker device identifier 
    const int mqttServerPort = 1883;                // broker mqtt port
    const int MQTTSendTimeoutMS = 10000;            // Timeout between MQTT publishes
    int status = WL_IDLE_STATUS;
    WiFiClient espClient;
    PubSubClient client(espClient);  
    
    void setup() { 
      Serial.begin(115200);
      delay(1000);
      client.setServer(mqttServer, mqttServerPort);    // Configure MQTT connexion
      unsigned long baseEventTime = millis();
      byte temperatureSamples = pow(2, osrs_t - 1);
      byte pressureSamples = pow(2, osrs_p - 1);
      byte humiditySamples = pow(2, osrs_h - 1);
      Serial.print ("Temperature samples=");
      Serial.print (temperatureSamples);
      Serial.print (" Pressure samples=");
      Serial.print (pressureSamples);
      Serial.print (" Humidity samples=");
      Serial.println (humiditySamples);
      bme0.begin(osrs_t, osrs_p, 1, 0, 0, 0, osrs_h);
    }
     
    void loop() {
      connectWifi();
      connectMQTT();
      measurementEvent();
     
      //delay(20000);
      
   }

   void measurementEvent() { 
        
      //*********** Measures Pressure, Temperature, Humidity, Voltage and calculates Altitude
      // then reports all of these to Blynk or Thingspeak
      
      while (bme0.readRegister(0xF3) >> 3); // loop until F3bit 3 ==0, measurement is ready
      double temperature, pressure;
      double humidity = bme0.readHumidity (temperature, pressure); // measure pressure, temperature and humidity
      float altitude = bme0.calcAltitude (pressure);
    
      Serial.print("Atm press = ");
      Serial.print(pressure, 2); // print with 2 decimal places
      Serial.print(" hPa. Temperature = ");
      Serial.print(temperature, 2); // print with 2 decimal places
      Serial.print( " deg C. Humidity = ");
      Serial.print(humidity, 2); // print with 2 decimal places
      Serial.print( " %RH. Altitude = ");
      Serial.print(altitude, 2); // print with 2 decimal places

      client.publish("/Wemostemp",String(temperature).c_str(),true);    //Send to MQTT
      client.publish("/Wemoshumid",String(humidity).c_str(),true);    //Send to MQTT
      client.publish("/Wemospres",String(pressure).c_str(),true);    //Send to MQTT
      client.publish("/Wemosalt",String(altitude).c_str(),true);    //Send to MQTT

      
    
      //******Battery Voltage Monitoring*********************************************
      
      // Voltage divider R1 = 220k+100k+220k =540k and R2=100k
      float calib_factor = 5.28; // change this value to calibrate the battery voltage
      unsigned long raw = analogRead(A0);
      float volt= raw * calib_factor/1024;      
      Serial.print( " m. \nVoltage = ");
      Serial.print(volt, 2); // print with 2 decimal places
      Serial.println (" V");
      client.publish("/Wemosvolt",String(volt).c_str(),true);    //Send to MQTT
      Serial.println(F("Data sent to MQTT broker "));
      Serial.println("Going into deep sleep mode for 10 minutes");
      delay(1000);
      ESP.deepSleep(600e6);    
   }
      
    void connectMQTT(){
      while (!client.connected()) {                 //if not connected: try to reconnect
        Serial.println("MQTT not connected: connecting...");
         if (client.connect(device, key, secret)) { //if not connected: try to reconnect
            Serial.println("connected to MQTT");
          } else {
            Serial.println("failed to connect to MQTT ");
            delay(2000);
            }
        }
    }
    
    void connectWifi(){                         // attempt to connect to Wifi network:
      while (WiFi.status() != WL_CONNECTED) {
        Serial.print("Wifi not connected: attempting to connect to SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
        delay(20000);                           // wait 20 seconds for connection:
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
