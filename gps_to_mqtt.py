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

import argparse
import threading
import os
from novatel_OEM4_python.NovatelOEM4 import Gps
from time import sleep
import signal
import logging
import paho.mqtt.client as mqtt
import sys
import struct

try:
    import queue
except ImportError:
    import Queue as queue

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


class MQTTHandler(logging.Handler):
    """
    A handler class which writes logging records, appropriately formatted,
    to a MQTT server to a topic.
    """

    def __init__(self, client, topic, qos=0, retain=False):
        logging.Handler.__init__(self)
        self.topic = topic
        self.qos = qos
        self.retain = retain
        self.client = client

    def emit(self, record):
        """
        Publish a single formatted logging record to a broker, then disconnect
        cleanly.
        """
        msg = self.format(record)
        self.client.publish(self.topic, payload=msg,
                            qos=self.qos, retain=self.retain)


class Controller:

    def __init__(self):
        self.gps = None  # type: Gps
        self.mqtt_client = None  # type: mqtt.Client
        self.mqqt_logger = None  # type: MQTTHandler
        # open files for storing data from sensors
        self.file_fp = open('gps.csv', 'w')
        self.gps_thread = None  # type: threading.Thread
        # event flag to exit
        self.exit_flag = threading.Event()  # type: threading.Event
        self.mqqt_online = False  # type: bool
        self.data_queue = queue.Queue()  # type: queue.Queue
        self.logger = logging.getLogger('GPS TO MQTT')  # type: logging.Logger
        # quick handlers for sending struct pack as binary.
        self.position = struct.Struct('<fff')
        self.velocity = struct.Struct('<fff')

    def handle_gps_data(self):
        """
        Function to be run in a thread to handle data from gps device and forward it to mqtt and save it to cvs file
        """
        if self.file_fp is None:
            self.log_info('GPS file not accessible. Is it initialized?')
            self.exit_flag.set()
        self.file_fp.write(
            "Index,Time,PSolStatus,X,Y,Z,stdX,stdY,stdZ,VSolStatus,VX,VY,VZ,stdVX,stdVY,stdVZ,\
            VLatency,SolAge,SolSatNumber\n")
        self.file_fp.flush()
        while not self.exit_flag.isSet():
            if not self.data_queue.empty():
                new_data = self.data_queue.get()
                self.file_fp.write('{0:5d},{1},{2},{3},{4},{5},'
                                   '{6},{7},{8},{9},{10},{11},'
                                   '{12},{13},{14},{15},{16},'
                                   '{17},{18}\n'.format(new_data['Index'],
                                                        new_data['Time'],
                                                        new_data['pSolStatus'],
                                                        new_data['position'][0],
                                                        new_data['position'][1],
                                                        new_data['position'][2],
                                                        new_data['positionStd'][0],
                                                        new_data['positionStd'][1],
                                                        new_data['positionStd'][2],
                                                        new_data['velSolStatus'],
                                                        new_data['velocity'][0],
                                                        new_data['velocity'][1],
                                                        new_data['velocity'][2],
                                                        new_data['velocityStd'][0],
                                                        new_data['velocityStd'][1],
                                                        new_data['velocityStd'][2],
                                                        new_data['vLatency'],
                                                        new_data['solAge'],
                                                        new_data['numSolSatVs']
                                                        ))
                self.file_fp.flush()
                message = self.position.pack(new_data['position'][0], new_data['position'][1], new_data['position'][2])
                self.mqtt_client.publish(ekf_topics['position'], payload=message, qos=0, retain=True)
                message = self.velocity.pack(new_data['velocity'][0], new_data['velocity'][1], new_data['velocity'][2])
                self.mqtt_client.publish(ekf_topics['velocity'], payload=message, qos=0, retain=True)
                # create high byte for velocity status and low byte for position status
                valid_message = int((new_data['velSolStatus'] << 8) | new_data['pSolStatus'])
                self.mqtt_client.publish(ekf_topics['valid'],
                                         payload=valid_message.to_bytes(4, 'little', signed=False),
                                         qos=0, retain=True)
            else:
                sleep(0.01)  # max log freq is 20Hz so it should be enough
        return

    def begin_gps(self, name='GPS', com_port=None):
        """
        Initialize gps device
        Args:
            name: name of device
            com_port: com port where device is connected
        Return
            bool: a boolean if device is correctly connected or not.
        """
        if com_port is None:
            self.log_info("Undefined port name")
            return False
        # create classes for sensors
        self.gps = Gps(sensorName=name)
        # declare threads
        self.gps_thread = threading.Thread(name="gps", target=self.handle_gps_data)

        if not self.gps.begin(self.data_queue, comPort=com_port):
            self.log_info("Failed to initialize device: {0}".format(self.gps.name))
            return False
        self.gps_thread.start()

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

    def log_info(self, message=None):
        """ Log a message

        A wrap around logging.
        The log message will have the following structure\:
        [class name \: function name ] message

        Args:
            message: a string with the message.
        """
        if message is None:
            # do nothing
            return
        self.logger.info('[{0}:{1}] {2}'.format(
            self.__class__.__name__,
            sys._getframe(1).f_code.co_name,
            message))
        return

    def log_debug(self, message=None):
        """ Log a message with debug level

        A wrap around logging.
        The log message will have the following structure\:
        [class name \: function name ] message

        the function name will be the caller function retrieved automatically
        by using sys._getframe(1).f_code.co_name

        Args:
            message: a string with the message.
        """
        if message is None:
            # do nothing
            return

        self.logger.debug('[{0}:{1}] {2}'.format(
            self.__class__.__name__,
            sys._getframe(1).f_code.co_name,
            message))
        return

    def clean_exit(self):
        """Handle exiting request

        Before exiting, send a message to mqtt broker to correctly signal the
        disconnection.
        The function must be appended as method to mqtt client object.
        """
        self.log_info("Requesting clean exit...")
        self.gps.shutdown()
        self.exit_flag.set()
        self.gps_thread.join()
        self.log_info("Successfully exited from devices")
        # are we connected to mqtt broker
        if self.mqqt_online:
            # tell we are disconnected on ekf topic
            (rc, _) = self.mqtt_client.publish(ekf_topics['connected'], payload=False.to_bytes(1, 'little'),
                                               qos=2, retain=True)
            if rc is not mqtt.MQTT_ERR_SUCCESS:
                logging.info('Failed to publish on exit: {0}'.format(ekf_topics['connected']))

            sleep(1)
            # wait for all messages are published before disconnect
            while len(self.mqtt_client._out_messages):
                sleep(0.01)
            self.mqtt_client.disconnect()
        return


def main():
    """
    :return:
    """
    global controller  # type: Controller

    def signal_handler(signum, frame):
        if signum == signal.SIGINT:
            controller.log_info('Received signal INTERRUPT... exiting now')
        if signum == signal.SIGTERM:
            controller.log_info('Received signal TERM... exiting now')
        controller.exit_flag.set()
        return

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            controller.mqtt_online = True
            # successfully connected
            (rc, _) = client.publish(ekf_topics['connected'], payload=True.to_bytes(1, 'little'),
                                     qos=2, retain=True)
            if rc is mqtt.MQTT_ERR_SUCCESS:
                # now add mqttLog to root logger to enable it
                logging.getLogger('').addHandler(controller.mqqt_logger)

                # TODO subscribe to other topics
            else:
                logging.info('Unexpected result on publish: rc={0}'.format(rc))
        else:
            logging.info("Failed to connect to server")
        return

    def on_disconnect(client, userdata, rc):
        if rc != 0:
            logging.info("Unexpected MQTT disconnection. Will auto-reconnect")
        controller.mqqt_online = False
        return

    # --------------------------------------------------------------------------
    #
    # Start of main part
    #
    # --------------------------------------------------------------------------
    if sys.version_info < (3, 0):
        print("Please use python version 3")
        return

    parser = argparse.ArgumentParser(add_help=True, description="Logger for two GPS units and a razor device")
    parser.add_argument("--gps_port", action="store", type=str,
                        dest="gps_port", default="/dev/ttyUSB0")
    parser.add_argument("-f", "--folder", action="store", type=str,
                        dest="folder", default="test1")
    parser.add_argument('--log', action='store', type=str, dest='log', default='main.log',
                        help='log file to be used')
    parser.add_argument("--log-level", action="store", type=str,
                        dest="logLevel", default='info',
                        help='Log level to be used. See logging module for more info',
                        choices=['critical', 'error', 'warning', 'info', 'debug'])
    parser.add_argument('--hostname', action='store', default='raspberrypi.local', type=str,
                        help='hostname for mqtt broker', dest='hostname')
    parser.add_argument('--port', action='store', default=8080, type=int,
                        help='port for mqtt broker', dest='port')
    parser.add_argument('--transport', action='store', default='websockets', type=str,
                        help='transport layer used in mqtt', dest='transport')
    args = parser.parse_args()

    log_level = {'error': logging.ERROR,
                 'debug': logging.DEBUG,
                 'info': logging.INFO,
                 'warning': logging.WARNING,
                 'critical': logging.CRITICAL
                 }
    # --------------------------------------------------------------------
    # create folder anc change dir
    # --------------------------------------------------------------------
    current_dir = os.getcwd()
    current_dir = current_dir + "/data/" + args.folder
    if not os.path.exists(current_dir):
        os.makedirs(current_dir)
    os.chdir(current_dir)
    # --------------------------------------------------------------------
    logging.basicConfig(filename=args.log,
                        level=log_level[args.logLevel],
                        format='[%(asctime)s] [%(name)-20s] [%(threadName)-10s] %(levelname)-8s %(message)s',
                        filemode="w")

    # ---------------------------------------------------------------------------
    # define a Handler which writes INFO messages or higher in console
    # ---------------------------------------------------------------------------
    console = logging.StreamHandler()
    console.setLevel(log_level[args.logLevel])
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-20s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)
    # initialize controller
    controller = Controller()
    # create mqtt client
    controller.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv311, transport=args.transport)
    # in case of lost connection, tell other we are dead.
    controller.mqtt_client.will_set(ekf_topics['connected'], payload=False.to_bytes(1, 'little'),
                                    qos=2, retain=True)
    # define callbacks for mqtt client
    controller.mqtt_client.on_connect = on_connect
    controller.mqtt_client.on_disconnect = on_disconnect

    if not controller.set_mqqt_logger(general_topics['log']):
        controller.log_info('Failed to set mqtt logger')

    no_faults = True
    try:
        controller.mqtt_client.connect(args.hostname, port=args.port)
        controller.mqtt_client.loop_start()
    except Exception as e:
        controller.log_info('Connection failed: {0}'.format(str(e)))
        no_faults = False
    finally:
        if not no_faults:
            controller.mqtt_client.loop_stop(force=True)
            controller.log_info('Failed to connect to broker...Exiting')
            return
    # configure gps unit
    controller.begin_gps(com_port=args.gps_port)
    # prepare signal and handlers
    signal.signal(signal.SIGINT, signal_handler)

    # send unlogall
    if not controller.gps.sendUnlogall():
        controller.log_info("Unlogall command failed on gps... check logfile")
        controller.clean_exit()
        logging.info('Exiting now')
        logging.shutdown()
        return

    # reconfigure port
    controller.gps.setCom(baud=115200)

    # set dynamics [0 air, 1 land, 2 foot]
    controller.gps.setDynamics(0)

    # enable augmented satellite systems
    # TODO: reporting bad checksum. Must be an issue on novatel oem library.
    # controller.gps.sbascontrol()

    # ask for bestxyz log at 20Hz
    controller.gps.askLog(trigger=2, period=0.05)
    # wait for Ctrl-C
    controller.log_info('Press Ctrl+C to Exit')
    signal.pause()
    # exit gracefully
    controller.clean_exit()
    controller.log_info('Exiting now')

    logging.shutdown()


if __name__ == '__main__':
    main()
