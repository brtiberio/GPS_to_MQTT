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

import logging
import argparse
import sys
import paho.mqtt.client as mqtt


def main():

    if sys.version_info < (3, 0):
        print("Please use python version 3")
        return

    parser = argparse.ArgumentParser(add_help=True,
                                     description='mqtt_logger')

    parser.add_argument('--hostname', action='store', default='raspberrypi.local', type=str,
                        help='hostname for mqtt broker', dest='hostname')
    parser.add_argument('--port', action='store', default=8080, type=int,
                        help='port for mqtt broker', dest='port')
    parser.add_argument('--transport', action='store', default='websockets', type=str,
                        help='transport layer used in ros bridge', dest='transport')
    parser.add_argument("--log-level", action="store", type=str,
                        dest="logLevel", default='debug',
                        help='Log level to be used. See logging module for more info',
                        choices=['critical', 'error', 'warning', 'info', 'debug'])

    args = parser.parse_args()
    log_level = {'error': logging.ERROR,
                 'debug': logging.DEBUG,
                 'info': logging.INFO,
                 'warning': logging.WARNING,
                 'critical': logging.CRITICAL
                 }

    hostname = args.hostname
    port = args.port
    transport = args.transport

    # ---------------------------------------------------------------------------
    # set up logging to file to used debug level saved to disk
    # ---------------------------------------------------------------------------
    logging.basicConfig(level=log_level[args.logLevel],
                        format='[%(asctime)s.%(msecs)03d] [%(name)-20s]: %(levelname)-8s %(message)s',
                        datefmt='%d-%m-%Y %H:%M:%S',
                        filename='mqtt_logger.log',
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

    mqtt_logger = logging.getLogger("mqtt logger")
    mqtt_logger.setLevel(log_level[args.logLevel])
    # formatter = logging.Formatter('[%(asctime)s.%(msecs)03d] [%(name)-20s]: %(levelname)-8s %(message)s')
    # mqtt_logger.setFormatter(formatter)

    client = mqtt.Client(protocol=mqtt.MQTTv311, transport=transport)
    client.enable_logger(mqtt_logger)

    try:
        logging.info('[Main]  Connecting to {0} on port {1}'.format(hostname, port))
        client.connect(hostname, port=port)
        logging.info('[Main]  subscribing to all VIENA topics...'.format(hostname, port))
        client.subscribe("VIENA/#", 0)
        client.loop_forever()
    except KeyboardInterrupt as e:
        logging.info('[Main] Got exception {0}... exiting now'.format(e))
    finally:
        client.loop_stop()
    return


if __name__ == '__main__':
    main()


