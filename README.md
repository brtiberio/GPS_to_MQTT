# GPS_to_MQTT

Simple scripts to test the interchange communication between GPS units to mqtt and then forward to rosbridge.

It is designed for python3 only. Not tested in python2.

Use requirements.txt to install the needed libraries.

## Working principle

File gps_to_mqtt runs a simple client that stores data from novatel library in a csv file and also sends position, velocity and validity of ECEF referential to
corresponding topic in mqtt. Data is sent as binary format.

Folder parameter defines the location to store data taking from base the ```./data/ folder```.

The file gps_mqtt_ros runs a bridge client that sends the data received from mqtt published by
gps_to_mqtt client and forward it into ros topics via rosbridge services.
Example of running:

```python
python gps_to_mqtt.py --gps_port=/dev/ttyUSB0 --folder=test1
````

Then use the gps_mqtt_ros to connect rosbridge server:

```python
python gps_mqtt_ros.py --hostname_mqtt=localhost --hostname_ros=localhost --port_ros=9090 --port_mqtt=8080
````

File mqqt_logger can be used to log all messages sent to mqtt client in topic "VIENA/#"

Documentation about submodules can be seen in:
https://novatel-oem4-python.readthedocs.io/en/latest/ and in https://mqtt-to-ros.readthedocs.io/en/latest/

It requires the use of a mqtt broker server. Can be used locally with mosquitto.
