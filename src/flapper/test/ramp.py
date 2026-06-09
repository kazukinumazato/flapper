#!/usr/bin/env python

# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
    #  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
# """
# Simple example that connects to the first Crazyflie found, ramps up/down
# the motors and disconnects.
# """
import logging
import time
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper
from pynput import keyboard


uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

logging.basicConfig(level=logging.ERROR)

class MotorRampExample:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Arm the Crazyflie
        self._cf.platform.send_arming_request(True)
        time.sleep(1.0)

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._ramp_motors).start() 

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

        
    def _ramp_motors(self):
        thrust_mult = 1
        thrust_step = 100
        default_thrust = 0
        thrust = default_thrust
        pitch = 0
        roll = 0
        yawrate = 0
        stop_flag = False

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # while thrust >= default_thrust:
        #     self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        #     time.sleep(0.1)
        #     if thrust >= default_thrust + 5000:
        #         thrust_mult = -1
        #     thrust += thrust_step * thrust_mult
        # while thrust < default_thrust - 5000 and thrust > default_thrust - 10000:
        #     self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        #     thrust -= thrust_step
        #     time.sleep(1)
        def on_press(key):
            nonlocal stop_flag
            nonlocal thrust
            nonlocal pitch
            nonlocal roll
            nonlocal yawrate
            print(f'Key pressed: {key}')
            try:
                if key.char is not None:
                    if key.char == "w":
                        pitch = 20
                    if key.char == "s":
                        pitch = -20
                    if key.char == "a":
                        roll = 10
                    if key.char == "d":
                        roll = -10
            except AttributeError:
                if key == keyboard.Key.up:
                    thrust += thrust_step
                if key == keyboard.Key.down and thrust >= thrust_step:
                    thrust -= thrust_step
                if key == keyboard.Key.right:
                    yawrate = -10
                if key == keyboard.Key.left:
                    yawrate = 10
                if key == keyboard.Key.esc:
                    stop_flag = True
                    return False  # Escキーが押されたらリスナーを停止
        def on_release(key):
            nonlocal pitch
            nonlocal roll
            nonlocal yawrate
            print(f'key released: {key}')
            try:
                if key.char is not None:
                    if key.char == "w" or  key.char == "s":
                        pitch = 0
                    if key.char == "a" or  key.char == "d":
                        roll = 0
            except AttributeError:
                if key == keyboard.Key.right or key == keyboard.Key.left:
                    yawrate = 0
                    
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        while True:
            print(f'thrust:{thrust} pitch:{pitch} roll:{roll} yawrate:{yawrate}')
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)
            if stop_flag == True:
                print("ループを終了します。")
                break
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(1)
        self._cf.close_link()


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = MotorRampExample(uri)
