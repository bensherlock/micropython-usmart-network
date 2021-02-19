#! /usr/bin/env python
#
# MicroPython TDA-MAC network protocol implementation for USMART sensor nodes
# This file is part of the following repository:
# https://github.com/bensherlock/micropython-usmart-network
#
# Standard Interface for NetProtocol
#   net_protocol = sensor_node.NetProtocol() # create an object in the main application loop
#   net_protocol.init_interfaces(modem_obj, sensor_obj) # link it with the modem and sensor payload objects
#   (can_go_to_sleep, time_till_next_req) = net_protocol.handle_packet(packet) # handle a received packet of structure "UN..."
#
# MIT License
#
# Copyright (c) 2020 Nils Morozs <nils.morozs@york.ac.uk>
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

# Import the standard pyBoard modules
import pyb
import machine
import struct
import utime

# Import the necessary modules
print("  Importing gateway/relay node function library...")
import uac_network.main.gw_functions as gwf
from sensor_payload.main.sensor_payload import SensorPayload
from uac_modem.main.unm3driver import MessagePacket, Nm3

print("  Imports within sensor_node.py completed")

### Define the network protocol class
class NetProtocol:
    
    # Constructor method including the initialization code
    def __init__(self):
        
        # Initialize the TDA-MAC parameters
        self.nm = None                      # reference to the modem object
        self.sensor  = None                 # sensor payload interface
        self.debug_flag = False             # set to True for more console output
        self.thisNode = -1                  # node address string to be queried from the nanomomodem
        self.masterNode = -1                # address of the master node for this node
        self.txDelay = 0                    # transmit delay for this node [ms]
        self.timeTillNextFrame = 0          # time till next frame [ms]
        self.ttnfTimestamp = 0              # time at which the time till next frame was saved
        self.subframeLength = 0             # subframe length for a relay node [ms]
        self.childNodes = list()            # list of child nodes for dual-hop TDA-MAC
        self.frameStartTime = 0             # start time of the current frame
        self.guardInt = 200                 # guard interval for timed Tx/Rx [ms]
        self.location = (0.0, 0.0)          # tuple to store the Lat-Long location
        self.wdt = None                     # WDT passed from mainloop to be fed during longer loops.
        print('Initializing the TDA-MAC parameters...')
        
    # Method to initialize the interfaces with the sensor payload and modem
    def init_interfaces(self, modem, sensor_payload, wdt=None):
    
        # Save the references to these objects
        self.nm = modem
        self.sensor = sensor_payload
        self.wdt = wdt

        # Feed the watchdog
        if self.wdt:
            self.wdt.feed()

        # Query this node's address and display it
        # addr = -1
        # while addr == -1: # This would lock up permanently.
        addr = modem.get_address()
        print("  This node's address: " + '%03d' % addr)
        self.thisNode = addr  # save the node's address

        # Feed the watchdog
        if self.wdt:
            self.wdt.feed()

        # Query this node's battery voltage and display it
        # voltage = -1
        # while voltage == -1: # This would lock up permanently.
        voltage = modem.get_battery_voltage()
        print("  Voltage supplied to the modem: " + '%0.2f' % voltage + "V")
        print(" ")
            
    ### Setter method for the estimated location
    def set_location(self, location):
        self.location = location
            
    ### General packet handling function that performs a particular action given the received packet
    # Returns (can_go_to_sleep, time_till_next_req)
    def handle_packet(self, packet):
            
        # Initialize the flag indicating whether the TDA-MAC frame is over after handling this packet
        canGoToSleep = False
        
        # Parse the received packet
        payload = bytes(packet.packet_payload)
        srcId = packet.source_address
        pktType = packet.packet_type
        
        # If in debug mode, display a message in terminal
        if self.debug_flag:
            print('    Rx from N' + str(packet.source_address) + ': ' + str(payload))

        # Feed the watchdog
        if self.wdt:
            self.wdt.feed()

        # Network discovery request packet
        if (pktType == 'U') and (len(payload) > 4) and (payload[0:4] == b'UNN?'):
            self.dealWithNetDiscReq(payload)
        # TDI packet unicast to this node
        elif (pktType == 'U') and (len(payload) > 4) and (payload[0:4] == b'UNI!'):
            self.dealWithTDIPacket(payload)
        # Broadcast REQ packet from the master node
        elif (pktType == 'B') and (srcId == self.masterNode) and (len(payload) > 5) and (payload[0:3] == b'UNR'):
            canGoToSleep = self.dealWithBroadcastREQ(srcId, payload, None)
        # Unicast REQ packet
        elif (pktType == 'U') and (len(payload) > 4) and (payload[0:3] == b'UNR'):
            canGoToSleep = self.dealWithUnicastREQ(payload)
        # General "Are you alive" packet
        elif (pktType == 'U') and (len(payload) == 11) and (payload[0:8] == b'UNalive?'):
            self.respondToDiagnosticPacket(payload)
        else:
            pass
            
        # If the current frame is over, node can go to sleep until the start of the next frame
        #ttnf = 0
        #if canGoToSleep:
        # Changed to always return the time to next frame
        ttnf = self.timeTillNextFrame - utime.ticks_diff(utime.ticks_ms(), self.ttnfTimestamp)
        return (canGoToSleep, ttnf)
    
    ##################################################################
    ### Function to deal with the network discovery request packet ###
    ##################################################################
    def dealWithNetDiscReq(self, payload):
        
        # Parse the source address (make sure it is three characters)
        srcNode = struct.unpack('B', payload[4:5])[0]
            
        # Parse the list of nodes that need to be discovered
        nodeList = list()
        for n in range(len(payload)-5):
            # Parse the address, make sure it is 3 characters
            addr = struct.unpack('B', payload[5+n:6+n])[0]
            # Append the address to the list 
            nodeList.append(addr)
            
        # Print message
        print('Node discovery request received from Node ' + str(srcNode))
            
        # Perform network discovery
        (propDelays, linkQuality) = gwf.doNetDiscovery(self.nm, nodeList, self.wdt)
            
        # Send the node discovery results packet to the requesting node
        self.sendNodeDiscPacket(srcNode, propDelays, linkQuality)
    
    ##########################################
    ### Function to deal with a TDI packet ###
    ##########################################
    def dealWithTDIPacket(self, payload):
        
        # Parse the source address
        srcNode = struct.unpack('B', payload[4:5])[0]
        # Parse the destination address
        destNode = struct.unpack('B', payload[5:6])[0]
        # Parse the Tx delay and subframe interval
        txd = struct.unpack('I', payload[6:10])[0]
        sfLength = struct.unpack('I', payload[10:14])[0]
        
        # Print message
        print('TDI received from Node ' + str(srcNode) + " for Node " + str(destNode) + ": " + str(txd) + ' ms')
            
        # If this TDI is addressed to me, update the Tx delay and the master node address
        tdiDelivered = False
        if destNode == self.thisNode:
            self.txDelay = txd # [ms]
            self.masterNode = srcNode
            tdiDelivered = True
        
        # Otherwise this TDI needs to be forwarded to the destination node
        else:           
            # First note the subframe length, because I will be the master node for this node
            self.subframeLength = sfLength # [ms] 
            
            # Try sending a TDI and receiving an ACK
            pyb.delay(100) # first, sleep long enough to transmit the Auto-ACK
            tdiDelivered = gwf.sendTDIPackets(self.nm, self.thisNode, [destNode], [txd], 0, [True], self.wdt)[0]
                            
            # Save the destination node as the child node
            if not (destNode in self.childNodes):
                self.childNodes.append(destNode)
        
        # Transmit TDI ACK/NACK packet
        if tdiDelivered:
            txMessage = b'UNIA' + struct.pack('B', self.thisNode)
            print('Sending TDI ACK...')
        else:
            txMessage = b'UNIN' + struct.pack('B', self.thisNode)
            print('Sending TDI NACK...')
        self.nm.send_unicast_message(srcNode, txMessage)
            
    ####################################################
    ### Function to deal with a Broadcast REQ packet ###
    ####################################################
    def dealWithBroadcastREQ(self, srcId, payload, dataPacket):
        
        # Start reading the sensor if the data packet is not given (new transmission)
        # mainloop is responsible for initiating sensor data acquisition and processing now.
        # if not dataPacket:
        #    self.sensor.start_acquisition()
        # Note the time of receiving this packet
        reqTime = utime.ticks_ms()
        
        # If this is the first broadcast REQ of the frame, note the frame start time
        reqIndex = struct.unpack('B', payload[3:4])[0]
            
        # Update the start of frame time
        self.timeTillNextFrame = struct.unpack('I', payload[4:8])[0]
        self.ttnfTimestamp = reqTime # note the time stamp for this TTNF
            
        # Check if I need to go back to always-on state after this
        sleepFlag = struct.unpack('B', payload[8:9])[0]
        
        # Decode the addresses of nodes expected to respond
        destAddr = list()
        for n in range(10, len(payload)):
            addr = struct.unpack('B', payload[n:n+1])[0]
            destAddr.append(int(addr))
            
        # Respond only if this node is in the list
        if self.thisNode in destAddr:
        
            # Print message for debugging
            print("REQ received from Node " + str(srcId))
            print("  Time till next frame: " + str(self.timeTillNextFrame) + " msec")
            
            # If this is a request for location, put it into the payload payload
            if payload[9:10] == b'L':
                # Create the data payload
                dataPayload = b'L' + struct.pack('f', self.location[0]) + b'L' + struct.pack('f', self.location[1])
                packetPayload = b'UND' + struct.pack('B', self.thisNode) + dataPayload
            
            # Otherwise, this is a request for sensor readings
            else:   
                # Read the sensor and create the data payload
                try:
                    if not dataPacket:
                        # mainloop is responsible for initiating sensor data acquisition and processing now.
                        # self.sensor.process_acquisition()
                        dataPayload = self.sensor.get_latest_data_as_bytes()
                        packetPayload = b'UND' + struct.pack('B', self.thisNode) + dataPayload
                    else:
                        packetPayload = dataPacket                       
                except Exception as e:
                    # If an Exception was caught, print the error
                    print("Error reading the sensor: " + str(e))
                    packetPayload = b'UND' + struct.pack('B', self.thisNode) + b'sensor_error'
            
            # Sleep for the remaining part of the transmit delay
            timeElapsed = utime.ticks_diff(utime.ticks_ms(), reqTime)
            if (self.txDelay > timeElapsed):
                pyb.delay(self.txDelay - timeElapsed)

            # Transmit the payload packet to the master node
            self.nm.send_unicast_message(srcId, packetPayload)
            
            # Print this transmission
            if payload[9:10] == b'L':
                print("Location data sent: Lat=" + '%.5f' % self.location[0] + ", Long=" + '%.5f' % self.location[1])
            else:
                print("Sensor readings sent")
            
            # If I have any child nodes,m do not go to sleep after this REQ    
            if self.childNodes:
                sleepFlag = 0
                
            # Wait for a retransmission request, if one arrives (10 sec timeout)
            reTxTimeout = 10000
            timerStart = utime.ticks_ms()
            while utime.ticks_diff(utime.ticks_ms(), utime.ticks_add(timerStart, reTxTimeout)) < 0:

                # Feed the watchdog
                if self.wdt:
                    self.wdt.feed()

                # Check if a packet has been received
                self.nm.poll_receiver()
                self.nm.process_incoming_buffer()
                if self.nm.has_received_packet(): 
      
                    # Read the incoming packet and see if it is a REQ
                    packet = self.nm.get_received_packet()
                    payload = bytes(packet.packet_payload)
                    srcId = packet.source_address
                    pktType = packet.packet_type
                    # If it is a REQ, process it by calling this function again
                    if (pktType == 'B') and (srcId == self.masterNode) and (len(payload) > 5) and (payload[0:3] == b'UNR'):
                        canGoToSleep = self.dealWithBroadcastREQ(srcId, payload, packetPayload)
                    # If it is a unicast REQ, data transmission was successful, move on to relaying
                    elif (pktType == 'U') and (len(payload) > 4) and (payload[0:3] == b'UNR'):
                        canGoToSleep = self.dealWithUnicastREQ(payload)
                    # Otherwise, pass it up to the main packet handling function
                    else:
                        canGoToSleep = self.handle_packet(packet)[0]
                    # Convert the sleep flag to an integer for internal consistency (due to recursion here!)
                    sleepFlag = 1 if (canGoToSleep) else 0
                    break
        
        # Return the flag indicating if I can go to sleep or should stay awake
        return (sleepFlag == 1)
    
    ##################################################
    ### Function to deal with a Unicast REQ packet ###
    ##################################################    
    def dealWithUnicastREQ(self, payload):
    
        # Note the time of receiving this packet
        reqTime = utime.ticks_ms()
        
        # Parse the source address
        srcNode = struct.unpack('B', payload[3:4])[0]
        
        # If this is the first REQ or a repeated REQ (retransmissions)
        reqIndex = struct.unpack('B', payload[4:5])[0]
            
        # Check if I need to go back to always-on state after this
        sleepFlag = struct.unpack('B', payload[5:6])[0]
        
        # Decode the addresses of nodes expected to respond
        destAddr = list()
        for n in range(7, len(payload)):
            addr = struct.unpack('B', payload[n:n+1])[0]
            destAddr.append(int(addr))
    
        # Print message for debugging
        print("Unicast REQ received from Node " + str(srcNode))
        
        # If this is a blank REQ (e.g. go-to-sleep instruction)
        if not destAddr:
            
            # Transmit a blank broadcast REQ to my child nodes three times
            numREQCopies = 3
            interval = 2000 # 2 second intervals between REQ copies
            for n in range(numREQCopies):

                # Feed the watchdog
                if self.wdt:
                    self.wdt.feed()

                # Transmit blank broadcast REQ
                ttnf = max(0, self.timeTillNextFrame - utime.ticks_diff(utime.ticks_ms(), self.ttnfTimestamp))
                print("Sending Blank Broadcast REQ...")
                gwf.sendBroadcastREQ(self.nm, "S", n+1, ttnf, True, [])        
                # Wait for a set interval before transmitting it again
                pyb.delay(interval)
                
        # If this is not a blank REQ gather the data from my child nodes
        else:
        
            # Try gathering the data from all child nodes
            packetBuffer = list()
            nodesToRespond = destAddr.copy()
            numRetries = 3
            for n in range(1, numRetries+1):

                # Feed the watchdog
                if self.wdt:
                    self.wdt.feed()

                # Transmit a broadcast REQ packet
                ttnf = max(0, self.timeTillNextFrame - utime.ticks_diff(utime.ticks_ms(), self.ttnfTimestamp))
                print("Sending Broadcast REQ...")
                gwf.sendBroadcastREQ(self.nm, payload[6:7].decode(), n, ttnf, (sleepFlag==1), nodesToRespond)
            
                # Go into a loop listening for payload packets from child nodes
                sfStartTime = utime.ticks_ms()
                while utime.ticks_diff(utime.ticks_ms(), utime.ticks_add(sfStartTime, self.subframeLength + self.guardInt)) < 0:

                    # Feed the watchdog
                    if self.wdt:
                        self.wdt.feed()

                    # Check if a packet has been received
                    self.nm.poll_receiver()
                    self.nm.process_incoming_buffer()
                    if self.nm.has_received_packet():
                        # Decode the packet
                        packet = self.nm.get_received_packet()
                        payload = bytes(packet.packet_payload)
                        # If the source address is one of the child nodes
                        srcAddr = struct.unpack('B', payload[3:4])[0]
                        if (payload[0:3] == b'UND') and (srcAddr in self.childNodes): 
                            # Store the packet in the forwarding buffer and take out the child node out of the list
                            print("  Data packet received from N" + "%03d" % srcAddr)
                            packetBuffer.append(packet)
                            nodesToRespond.remove(srcAddr)
                                
                    # Add a delay before checking the serial port again     
                    pyb.delay(25)
                    
                # If there are no more child nodes to gather data from, do not transmit a REQ again
                if not nodesToRespond:
                    break
                    
            # Transmit a START DATA TRANSFER handshake packet to the gateway and wait for an ACK
            numTries = 5
            timeout = 5.0
            gwReady = False
            for n in range(numTries):
                # Feed the watchdog
                if wdt:
                    wdt.feed()  
                # Transmit a short handshake packet "UNSDT", if ACK is received, proceed with data transfer
                print("Contacting GW to initiate data transfer...")
                delay = nm.send_unicast_message_with_ack(srcNode, b'UNSDT', timeout)
                if delay > 0:
                    print("  GW is ready to receive")
                    gwReady = True
                    break
                
            # Forward all payload packets in the buffer to the node that requested it
            # Wait for a Repeated REQ in case retransmissions are required
            frameIsOver = False
            while gwReady and (not frameIsOver):
            
                # Forward the packets 
                for fwPacket in packetBuffer:

                    # Feed the watchdog
                    if self.wdt:
                        self.wdt.feed()

                    # Send the packet
                    payload = bytes(fwPacket.packet_payload)
                    srcAddr = struct.unpack('B', payload[3:4])[0]
                                   
                    # Transmit the payload packet if this node was specified in the REQ
                    if (srcAddr in destAddr):
                        print("Forwarding data packet from Node " + str(srcAddr) + " to Node " + str(srcNode) + "...") 
                        self.nm.send_unicast_message(srcNode, payload)
                        pyb.delay(gwf.dataPktDur + self.guardInt) # delay while we are transmitting the packet
                    
                # If there are any child nodes who did not respond with a data packet
                for unrNode in nodesToRespond:
                    if unrNode in destAddr:

                        # Feed the watchdog
                        if self.wdt:
                            self.wdt.feed()

                        # Send blank packets to the Gateway
                        payload = b'UND' + struct.pack('B', unrNode) + b'no_packet'
                        print("Sending blank data packet from Node " + str(unrNode) + " to Node " + str(srcNode) + "...") 
                        self.nm.send_unicast_message(srcNode, payload)
                        pyb.delay(gwf.dataPktDur + self.guardInt) # delay while we are transmitting the packet
                        
                # Wait for a repeated REQ asking for retransmissions (10 sec timeout)
                txEndTime = utime.ticks_ms()
                timeout = 10000
                anotherREQReceived = False
                while utime.ticks_diff(utime.ticks_ms(), utime.ticks_add(txEndTime, timeout)) < 0:

                    # Feed the watchdog
                    if self.wdt:
                        self.wdt.feed()

                    # Check if a packet has been received
                    self.nm.poll_receiver()
                    self.nm.process_incoming_buffer()
                    if self.nm.has_received_packet():       
                        # Read the incoming packet and see if it is a REQ
                        packet = self.nm.get_received_packet()
                        payload = bytes(packet.packet_payload)
                        srcId = packet.source_address
                        pktType = packet.packet_type
                        # If it is a REQ, resend some of the packets again
                        if (pktType == 'U') and (len(payload) > 4) and (payload[0:3] == b'UNR'):
                        
                            # Check if I need to go back to always-on state after this
                            anotherREQReceived = True
                            sleepFlag = struct.unpack('B', payload[5:6])[0]
                            
                            # Decode the addresses of nodes expected to respond
                            destAddr = list()
                            for n in range(7, len(payload)):
                                addr = struct.unpack('B', payload[n:n+1])[0]
                                destAddr.append(int(addr))
                                # Transmit the missing packets again (by staying in this loop)
                        # Otherwise, pass it up to the main packet handling function
                        else:
                            canGoToSleep = self.handle_packet(packet)[0]
                            sleepFlag = 1 if (canGoToSleep) else 0
                
                # Check if the frame is over
                frameIsOver = not anotherREQReceived
                
        # Return the sleep flag
        return (sleepFlag == 1)
                    
    
    ##################################################################
    ### Function to respond to the general diagnostic packet       ###  
    ##################################################################
    def respondToDiagnosticPacket(self, payload):
        
        # Read the source address formatted as three characters
        srcAddr = int(bytes(payload[8:11]).decode())        
        # Print message for debugging
        print("Diagnostic packet received from Node " + str(srcAddr));  
        # Convert own address to a three character string
        addrString = "%03d" % self.thisNode
        
        # Transmit a response
        txBytes = b'Yes!' + addrString.encode()
        nm.send_unicast_message(srcAddr, txBytes)
    
    ##############################################################################################  
    ### Function to deliver the network discovery results back to the node that requested them ###
    ##############################################################################################
    def sendNodeDiscPacket(self, reqNode, propDelays, linkQuality):
        
        # By default assume it is unsuccessful
        success = False
        
        # Create the payload packet containing all measured propagation delays and link qualities
        packet = b'UNNR'
        for n in range(len(propDelays)):
            packet += struct.pack('I', propDelays[n]) + struct.pack('B', linkQuality[n])
        
        # Try sending the packet multiple times if needed
        maxTries = 5 # maximum attempts at sending it
        timeout = 5 # 5 second ACK timeout
        for k in range(maxTries):

            # Feed the watchdog
            if self.wdt:
                self.wdt.feed()

            # Send the packet
            print("Sending node discovery results to Node " + str(reqNode))
            # Transmit the packet requiring an ACK
            delay = self.nm.send_unicast_message_with_ack(reqNode, packet, timeout)     
                
            # If the ACK was received
            if delay > 0:           
                # Print message
                print("  ACK received")
                # Success, we are done here
                success = True
                break
            # Otherwise, try sending the packet again
            else:
                print("  No ACK")
                        
        # Return the success flag
        return success
