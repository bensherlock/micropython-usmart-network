#! /usr/bin/env python
#
# MicroPython MainLoop for testing the USMART Sensor Application.
#
# Standard Interface for MainLoop
# - def run_mainloop() : never returns
#
# MIT License
#
# Copyright (c) 2020 Nils Morozs <nils.morozs@york.ac.uk>
# Adapted from Ben Sherlock's mainloop.py available at:
# https://github.com/bensherlock/micropython-usmart-sensor-mainloop
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
#
""" Test version of the MicroPython MainLoop for USMART Sensor Application."""

import pyb
import machine
import utime

# Import the protocol, sensor payload and nanomodem modules
print("Importing sensor payload package...")
import sensor_payload.main.sensor_payload as sensor_payload
print("Importing network protocol package...")
import uac_network.main.sensor_node as sensor_node
print("Importing NM3 library...")
from uac_modem.main.unm3driver import MessagePacket, Nm3
print("Imports completed")

# Define the possible states of the node
STATE_ALWAYS_ON = 0
STATE_PERIODIC_SLEEP = 1

# Standard Interface for MainLoop
# - def run_mainloop() : never returns
def run_mainloop():
    """Standard Interface for MainLoop. Never returns."""
    
    # Initialize the node state as always on
    node_state = STATE_ALWAYS_ON
   
    # Wait for 6 seconds to let the modem start up
    print("6 second delay to allow the NM3 to boot up...")
    pyb.delay(6*1000)

    # Initialize UART and NM3 object
    uart = machine.UART(1, 9600, bits=8, parity=None, stop=1, timeout=1000)
    modem = Nm3(uart)
    
    # Generate the sensor payload object
    sensor_pl = sensor_payload.get_sensor_payload_instance()

    # Create and initialize the network protocol object
    net_protocol = sensor_node.NetProtocol()
    net_protocol.init_interfaces(modem, sensor_pl)

    # Start an infinite loop, listen to packets and respond
    while True:
        
        # Check if a packet has been received
        modem.poll_receiver()
        modem.process_incoming_buffer()
        if modem.has_received_packet():
            # If it has, process it and pass it on to the TDA-MAC protocol handler
            packet = modem.get_received_packet()
            # Here I assume that this packet is definitely for Networking: "UN..."
            # In the full main loop, we will direct "UL..." and "UN..." packets to their respective modules here.
            # After localisation is finished, it will be easiest to return the (Lat, Lon) here, 
            # e.g. returned by Rahul's localisation function called in this loop.
            # It then can be passed to the Network Protocol object using:
            # net_protocol.set_location(lat, lon)
            # This will allow the node locations to be collected later using TDA-MAC, at the start of the data gathering stage
            
            # If the packet is "UN...", call this function handling all networking packets
            # It returns a flag stating whether the node can go to a sleep state until the next frame,
            # and the time [msec] until the next expected REQ packet from the master node (gateway or relay)
            (can_go_to_sleep, time_till_next_req) = net_protocol.handle_packet(packet)
            
            # If the TDA-MAC initialization has finished and the node can go to sleep, update the node state
            if (can_go_to_sleep):
                node_state = STATE_PERIODIC_SLEEP
            else:
                node_state = STATE_ALWAYS_ON

        # In this test, do not put the node to sleep
        # [TO INCORPORATE SLEEP MODE]
        # if (node_state == STATE_PERIODIC_SLEEP), 
        #       Put the node to sleep, but wake up (at the latest) after "time_till_next_req" msec
        #                              MINUS the time needed to power up all the modules (e.g. 6000 msec)
        
        # In this test, simply keep checking the input in 20 ms time increments
        pyb.delay(20)