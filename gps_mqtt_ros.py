#!/usr/bin/python
# -*- coding: utf-8 -*-
# The MIT License (MIT)
# Copyright (c) 2019 Bruno Tibério
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import roslibpy
import logging
import argparse
import sys
import signal
import paho.mqtt.client as mqtt
from time import sleep
from struct import unpack
from mqtt_to_ros.ros_mqtt_bridge import MqttRosBridge, MQTTHandler

# import pydevd
# pydevd.settrace('localhost', port=8000, stdoutToServer=True, stderrToServer=True)

# General Topics
general_topics = {'canopen':  'VIENA/General/canopen',  # canopen status
                  'rpi':      'VIENA/General/rpi',      # rpi client connected
                  'log':      'VIENA/General/log',      # logger topic
                  'mqtt_ros': 'VIENA/General/mqtt_ros'  # mqtt to ros bridge status
                  }
# GPS Topics TODO: complete temporary topics! Will be changed in future
ekf_topics = {'valid':     'VIENA/EKF/valid',     # computation data is valid
              'position':  'VIENA/EKF/position',  # send vector with X, Y, Z, ECEF position values
              'velocity':  'VIENA/EKF/velocity',  # send vector with VX, VY, VZ ECEF velocity values
              'connected': 'VIENA/EKF/connected'  # send status of connection
              }

# SINAMICS Topics
sinamics_topics = {'connected':             'VIENA/SINAMICS/connected',  # inverter connected status
                   'velocity':              'VIENA/SINAMICS/velocity',  # estimated velocity
                   'state_read':            'VIENA/SINAMICS/state/read',  # state from inverter to others
                   'state_write':           'VIENA/SINAMICS/state/write',  # state from others to inverter
                   'EMCY':                  'VIENA/SINAMICS/EMCY',  # print emergency messages
                   'target_velocity_read':  'VIENA/SINAMICS/target_velocity/read',  # target velocity read
                   'target_velocity_write': 'VIENA/SINAMICS/target_velocity/write',  # target velocity write
                   }


class SimpleController(MqttRosBridge):
    """
    A simple controller class example to be used with for testing purposes.
    Uses the subclass MqttRosBridge.
    Must implement the desired function callbacks for use with ros subscribed topics
    """

    def __init__(self, debug=False):
        super().__init__(debug)
        self.mqtt_logger = None  # type: MQTTHandler
        return

    def set_mqqt_logger(self, topic):
        if self.mqtt_client is None:
            self.log_info('No mqtt client defined')
            return False
        self.mqqt_logger = MQTTHandler(self.mqtt_client, topic)
        # save all levels
        self.mqqt_logger.setLevel(logging.INFO)
        self.mqqt_logger.setFormatter(logging.Formatter(
            fmt='[%(asctime)s.%(msecs)03d] [%(name)-20s]: %(levelname)-8s %(message)s',
            datefmt='%d-%m-%Y %H:%M:%S'))
        return True

    def clean_exit(self):
        """Handle exiting request

        Before exiting, send a message to mqtt broker to correctly signal the
        disconnection.
        The function must be appended as method to mqtt client object.
        """
        # tell we are disconnected on canopen topic
        (rc, _) = self.client_mqtt.publish(general_topics['canopen'], payload=False.to_bytes(1, 'little'),
                                           qos=2, retain=True)
        if rc is not mqtt.MQTT_ERR_SUCCESS:
            logging.info('Failed to publish on exit: {0}'.format(general_topics['canopen']))

        # tell we are disconnected on rpi topic
        (rc, _) = self.client_mqtt.publish(general_topics['rpi'], payload=False.to_bytes(1, 'little'),
                                           qos=2, retain=True)
        if rc is not mqtt.MQTT_ERR_SUCCESS:
            logging.info('Failed to publish on exit: {0}'.format(general_topics['rpi']))
        sleep(1)
        # wait for all messages are published before disconnect
        while len(self.client_mqtt._out_messages):
            sleep(0.01)
        self.client_mqtt.disconnect()
        # give some time to forward all messages to ros topics
        sleep(1)
        self.client_ros.close()
        return

    # ---------------------------------------------------------------------------
    # Defines of callback functions
    # ---------------------------------------------------------------------------
    def send_target_velocity(self, message):
        """
        Callback for received target velocity. Currently message is received as a string.
        Value must be a signed int32 compatible value

        Args:
            message: ros message received.
        Return:
            a boolean if correctly forwarded or not.
        """
        self.log_info('Received target velocity: {0}'.format(message['data']))
        try:
            var = int(message['data'])
        except ValueError:
            self.log_info("Not a valid value received: {0}".format(message['data']))
            return False
        # forward it as signed int32 to corresponding topic
        self.client_mqtt.publish(sinamics_topics['target_velocity_write'],
                                 payload=var.to_bytes(4, 'little', signed=True))
        return True


def main():
    global controller
    # ---------------------------------------------------------------------------
    # define signal handlers for systemd signals
    # ---------------------------------------------------------------------------

    def on_message(client, userdata, message):
        # TODO: define more messages. Currently for testing is only used the velocity
        controller.log_debug("Received message :" + str(message.payload) + " on topic "
                             + message.topic + " with QoS " + str(message.qos))
        if message.topic == sinamics_topics['velocity']:
            number = int.from_bytes(message.payload, 'little', signed=True)
            ros_message = roslibpy.Message({'data': str(number)})
            controller.mqtt_subscribers[sinamics_topics['velocity']]['ros_topic'].publish(ros_message)
        elif message.topic == ekf_topics['connected']:
            status = int.from_bytes(message.payload, 'little', signed=False)
            ros_message = roslibpy.Message({'data': bool(status)})
            controller.mqtt_subscribers[ekf_topics['connected']]['ros_topic'].publish(ros_message)
        elif message.topic == ekf_topics['valid']:
            status = int.from_bytes(message.payload, 'little', signed=False)
            ros_message = roslibpy.Message({'data': (not status)})
            controller.mqtt_subscribers[ekf_topics['valid']]['ros_topic'].publish(ros_message)
        elif message.topic == ekf_topics['position']:
            position = unpack('<fff', message.payload)
            ros_message = roslibpy.Message({'x': position[0], 'y': position[1], 'z': position[2]})
            controller.mqtt_subscribers[ekf_topics['position']]['ros_topic'].publish(ros_message)
        elif message.topic == ekf_topics['velocity']:
            velocity = unpack('<fff', message.payload)
            ros_message = roslibpy.Message({'x': velocity[0], 'y': velocity[1], 'z': velocity[2]})
            controller.mqtt_subscribers[ekf_topics['velocity']]['ros_topic'].publish(ros_message)

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            controller.mqtt_online = True
            # successfully connected
            while not controller.client_ros.is_connected:
                sleep(1)

            message = roslibpy.Message({'data': 'connected'})
            # controller.client_ros.publish(general_topics['mqtt_ros'], message)
            # now add mqttLog to root logger to enable it
            controller.set_mqqt_logger(general_topics['log'])
            logging.getLogger('').addHandler(controller.mqtt_logger)
            # TODO subscribe to other topics
        else:
            controller.log_info('Unexpected result on publish: rc={0}'.format(rc))
        return

    def on_disconnect(client, userdata, rc):
        if rc != 0:
            controller.log_info("Unexpected MQTT disconnection. Will auto-reconnect")
        controller.mqtt_online = False

    # ---------------------------------------------------------------------------
    # end of callback defines
    # ---------------------------------------------------------------------------

    if sys.version_info < (3, 0):
        print("Please use python version 3")
        return

    parser = argparse.ArgumentParser(add_help=True,
                                     description='ros_publish')

    parser.add_argument('--hostname_ros', action='store', default='localhost', type=str,
                        help='hostname for ros_bridge', dest='hostname_ros')
    parser.add_argument('--port_ros', action='store', default=9090, type=int,
                        help='port for ros bridge', dest='port_ros')
    parser.add_argument('--hostname_mqtt', action='store', default='raspberrypi.local', type=str,
                        help='hostname for mqtt broker', dest='hostname_mqtt')
    parser.add_argument('--port_mqtt', action='store', default=8080, type=int,
                        help='port for mqtt broker', dest='port_mqtt')
    parser.add_argument('--transport', action='store', default='websockets', type=str,
                        help='transport layer used in ros bridge', dest='transport')
    parser.add_argument("--log-level", action="store", type=str,
                        dest="logLevel", default='info',
                        help='Log level to be used. See logging module for more info',
                        choices=['critical', 'error', 'warning', 'info', 'debug'])

    args = parser.parse_args()
    log_level = {'error': logging.ERROR,
                 'debug': logging.DEBUG,
                 'info': logging.INFO,
                 'warning': logging.WARNING,
                 'critical': logging.CRITICAL
                 }

    hostname_ros = args.hostname_ros
    hostname_mqtt = args.hostname_mqtt
    port_ros = args.port_ros
    port_mqtt = args.port_mqtt
    transport = args.transport

    # ---------------------------------------------------------------------------
    # set up logging to file to used debug level saved to disk
    # ---------------------------------------------------------------------------
    logging.basicConfig(level=log_level[args.logLevel],
                        format='[%(asctime)s.%(msecs)03d] [%(name)-20s]: %(levelname)-8s %(message)s',
                        datefmt='%d-%m-%Y %H:%M:%S',
                        filename='mqtt_controller.log',
                        filemode='w')
    # ---------------------------------------------------------------------------
    # define a Handler which writes INFO messages or higher in console
    # ---------------------------------------------------------------------------
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-20s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)
    # create main controller
    controller = SimpleController()
    # create mqtt client
    controller.client_mqtt = mqtt.Client(protocol=mqtt.MQTTv311, transport=transport)
    # set callbacks for mqtt
    controller.client_mqtt.on_connect = on_connect
    controller.client_mqtt.on_message = on_message
    controller.client_mqtt.on_disconnect = on_disconnect
    controller.client_mqtt.clean_exit = controller.clean_exit

    # create ros client
    controller.begin_ros_client(hostname_ros, port_ros)
    # run ros client non-blocking
    controller.client_ros.run()
    controller.add_mqtt_to_ros(general_topics['mqtt_ros'], 'std_msgs/Bool', general_topics['mqtt_ros'])
    sleep(1)
    no_faults = True
    try:
        controller.client_mqtt.connect(hostname_mqtt, port=port_mqtt)
        controller.client_mqtt.loop_start()
    except Exception as e:
        logging.info('Connection failed: {0}'.format(str(e)))
        no_faults = False
    finally:
        if not no_faults:
            controller.client_mqtt.loop_stop(force=True)
            logging.info('Failed to connect to broker...Exiting')
            return

    # ---------------------------------------------------------------------------
    # create ros to mqtt for target velocity
    # orders come from ros, so it must be subscribed to write topic
    controller.add_ros_to_mqtt(sinamics_topics['target_velocity_write'], 'std_msgs/String',
                               sinamics_topics['target_velocity_write'], controller.send_target_velocity)
    # create mqtt to ros for velocity
    controller.add_mqtt_to_ros(sinamics_topics['velocity'], 'std_msgs/String', sinamics_topics['velocity'])

    # ------------------------------------------------------------------------------
    # EKF related
    # ------------------------------------------------------------------------------
    # create mqtt to ros for ekf connected
    controller.add_mqtt_to_ros(ekf_topics['connected'], 'std_msgs/Bool', ekf_topics['connected'])
    # create mqtt to ros for ekf valid
    controller.add_mqtt_to_ros(ekf_topics['valid'], 'std_msgs/Bool', ekf_topics['valid'])
    # create mqtt to ros for ekf position
    controller.add_mqtt_to_ros(ekf_topics['position'], 'geometry_msgs/Point32', ekf_topics['position'])
    # create mqtt to ros for ekf velocity
    controller.add_mqtt_to_ros(ekf_topics['velocity'], 'geometry_msgs/Vector3', ekf_topics['velocity'])
    # twisted overrides default signal handles, redirect it to  defined signal_handler function
    signal.signal(signal.SIGINT, signal.default_int_handler)
    # signal.signal(signal.SIGINT, signal_handler)
    # signal.signal(signal.SIGTERM, signal_handler)

    logging.info('waiting a bit to connect to server...')
    sleep(2)

    try:
        print("Ctrl+C to exit... ")
        while True:
            if not controller.client_ros.is_connected:
                controller.log_info('Not connected!')
                controller.client_ros.connect()
            sleep(1)
    except KeyboardInterrupt as e:
        logging.info('[Main] Got exception {0}... exiting now'.format(e))
    finally:
        controller.client_ros.terminate()
        controller.clean_exit()
        # wait for everything to be done
        sleep(1)
    return


if __name__ == '__main__':
    main()
