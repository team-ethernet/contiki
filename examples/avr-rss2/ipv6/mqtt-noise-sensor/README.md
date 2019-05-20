MQTT Noisesensor
=========
Mqtt noise sensor is based on MQTT GreenIot, but modified to send noise data from a SEN0232 sound meter. 

MQTT client
----------
The MQTT client can be used to:

* Publish sensor readings to an MQTT broker.
* Subscribe to a topic and receive commands from an MQTT broker

The demo will give some visual feedback with the green LED:
* Very fast blinking: Searching for a network
* Fast blinking: Connecting to broker
* Slow, long blinking: Sending a publish message

Publishing
----------
By default the example will attempt to publish readings to an MQTT broker
running on the IPv6 address specified as `MQTT_DEMO_BROKER_IP_ADDR` in
`project-conf.h`. This functionality was tested successfully with
[mosquitto](http://mosquitto.org/).

The publish messages include sensor readings but also some other information,
such as device uptime in seconds and a message sequence number. The demo will
publish to topic `noisesensor/<device-id>/sensors`. The device will connect using
client-id `d:quickstart:cc2538:<device-id>`, where `<device-id>` gets
constructed from the device's IEEE address.

Use
-----------
To compile and run the code with toolchain

`sudo make TARGET=avr-rss2`

and then

`sudo avrdude -p m256rfr2 -c stk500v2 -P /dev/<USB-port> -b 115200 -e -U flash:w:mqtt-noise-sensor.avr-rss2`


Modified by
-----------
Erik Flink \
Isak Olsson \
Nelly Friman \
Anton Bothin  \
Andreas Sjödin \
Jacob Klasmark  \
Carina Wickström \
Valter Lundegårdh
