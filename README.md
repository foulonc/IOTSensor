# IOTSensor
## Logic
* Initialize libraries
* Initialize variables (import secrets from secrets.h)
* set Error led on before first wifi connection
* loop
  * collect temperature reading
  * check wifi connection
    * not connected? connect
  * check MQTT connection
    * not connected? connect
  * publish measurement to MQTT
![Test Image 7](https://i.imgur.com/ht9y0QM.png)
