#! /usr/bin/env python
#
# MicroPython MainLoop for testing the USMART Gateway Application.
#
# Standard Interface for MainLoop
# - def run_mainloop() : never returns
#
# MIT License
#
# Copyright (c) 2020 Nils Morozs <nils.morozs@york.ac.uk>
# Adapted from Ben Sherlock's mainloop.py available at:
# https://github.com/bensherlock/micropython-usmart-gateway-mainloop
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
""" Test version of the MicroPython MainLoop for USMART Gateway Application."""

import pyb
import machine
import utime

# Import the protocol, sensor payload and nanomodem modules
print("Importing expansion board modules...")
from pybd_expansion.main.max3221e import MAX3221E
from pybd_expansion.main.powermodule import PowerModule
print("Importing network protocol package...")
import uac_network.main.gw_node as gw_node
print("Importing NM3 library...")
from uac_modem.main.unm3driver import MessagePacket, Nm3
print("Imports completed")

# Standard Interface for MainLoop
# - def run_mainloop() : never returns
def run_mainloop():
    """Standard Interface for MainLoop. Never returns."""
    
    # List of sensor node addresses expected to be in the network
    node_addr = [2, 3]
    
    # Data gathering time parameters [Frequenct data gathering and network discovery for testing]
    data_gathering_interval = 60*1000 # [msec]
    net_discovery_interval = 3 # (re)do network discovery every 3 cycles    
    
    # Enable the NM3 power supply on the powermodule
    powermodule = PowerModule()
    powermodule.enable_nm3()

    # Enable power supply to 232 driver
    pyb.Pin.board.EN_3V3.on()
    pyb.Pin('Y5', pyb.Pin.OUT, value=0)  # enable Y4 Pin as output
    max3221e = MAX3221E(pyb.Pin.board.Y5)
    max3221e.tx_force_on()  # Enable Tx Driver
   
    # Wait for 6 seconds to let the modem start up
    print("6 second delay to allow the NM3 to boot up...")
    pyb.delay(6*1000)

    # Initialize UART and NM3 object
    uart = machine.UART(1, 9600, bits=8, parity=None, stop=1, timeout=1000)
    modem = Nm3(uart)

    # Create and initialize the network protocol object
    net_protocol = gw_node.NetProtocol()
    net_protocol.init(modem, node_addr)
    
    # Start by doing the network discovery and setup
    net_protocol.do_net_discovery()
    net_protocol.setup_net_schedule() # guard interval [msec] can be specified as function input (default: 500)
    
    # Extract network topology and schedule information as JSON
    net_info = net_protocol.get_net_info_json()
    print(net_info) # print it in this test script (send over Wi-Fi in the real app)

    # Start an infinite loop, gathering sensor data
    cycle_counter = 0
    while True:
        
        # Perform a cycle of data gathering
        cycle_counter += 1
        frame_start_time = utime.ticks_ms() # update the frame start time
        stay_awake = (cycle_counter == net_discovery_interval) # if this is the last cycle before network re-discovery
        time_till_next_frame = data_gathering_interval # for sleep synchronisation (this can also be variable between frames)
        packets = net_protocol.gather_sensor_data(time_till_next_frame, stay_awake)
        # A list of MessagePacket objects is returned, to be processed and transmitted over Wi-Fi
        
        # If this was the last cycle before network re-discovery
        if (cycle_counter == net_discovery_interval):
            # Do network discovery and setup again (the network should be awake now)
            net_protocol.do_net_discovery()
            net_protocol.setup_net_schedule()
            cycle_counter = 0
            # When finished, instruct the network to sleep until the next frame
            time_till_next_frame = data_gathering_interval - utime.ticks_diff(utime.ticks_ms(), frame_start_time)
            net_protocol.set_network_to_sleep(time_till_next_frame)
            # Extract network topology and schedule information as JSON
            net_info = net_protocol.get_net_info_json()
            print(net_info) # print it in this test script (send over Wi-Fi in the real app)

        # Go to sleep yourself until the start of next frame
        # [This will need to be replaced by a proper sleep mode (with powered down modules)]
        time_till_next_frame = data_gathering_interval - utime.ticks_diff(utime.ticks_ms(), frame_start_time)
        pyb.delay(time_till_next_frame)
        
