#! /usr/bin/env python
#
# MicroPython TDA-MAC network protocol implementation for USMART sensor nodes
# This file is part of the following repository:
# https://github.com/bensherlock/micropython-usmart-network
#
# Standard Interface for NetProtocol
#   net_protocol = gw_node.NetProtocol()            # create an object in the main application loop
#   net_protocol.init(modem, node_addr)             # initialise it with the modem object and list of sensor node addresses
#   net_protocol.do_net_discovery(full_rediscovery=false) # perform network discovery
#       # One optional function input:
#       #   - binary flag indicating whether to force a full network rediscovery
#   net_protocol.setup_net_schedule(guard_int=500)  # set up TDA-MAC schedule and distribute it to all nodes
#   net_protocol.get_net_info_json                  # get network info as JSON
#   net_protocol.set_network_to_sleep(time_till_next_frame) # instruct all nodes to go to sleep for a given period [msec]
#   packets = net_protocol.gather_sensor_data(time_till_next_frame, stay_awake, data_type="S") #  perform a data gathering cycle 
#       # The function above takes two inputs: 
#       #    - time until the next frame [msec],
#       #    - flag indicating whether sensor nodes should stay awake after this cycle (e.g. for network re-discovery)
#       #    - A character specifying the data to be gathered: "L" - location, "S" - sensor data
#       # Function output:
#       #    - list of MessagePacket objects containing data packets gathered in this cycle
#
# MIT License
#
# Copyright (c) 2020 Nils Morozs <nils.morozs@york.ac.uk>
# Copyright (c) 2020 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>
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
print("  Importing gateway node function library...")
import uac_network.main.gw_functions as gwf
from uac_modem.main.unm3driver import MessagePacket, Nm3

print("  Imports within gw_node.py completed")

### Define the network protocol class
class NetProtocol:
    
    # Constructor method including the initialization code
    def __init__(self):

        # GW node attributes
        self.thisNode = -1          # node address to be queried from the nanomomodem
        self.relayLoads = None      # a running record of relay loads on each node (to even it out long term)
        self.guardInt = 500         # 500 ms guard interval (to be safe)
        self.lqThreshold = 4        # link quality thredhold (only use links of at least this quality if possible)
        self.debugFlag = False      # set to True for more console output
        self.dualHop = True         # enable dual-hop networking by default
        self.missingLinks = None    # a record of links that had no ping response since last full network discovery
        self.dataPacketSR = None    # data packet success ratio during the latest data gathering cycle
        self.srThreshold = 0.9      # data packet success ratio threshold (avoid retesting nodes above this)
        self.lowSrThreshold = 0.1   # data packet success ratio threshold (avoid retesting nodes below this)
        self.frameCounter = 0       # keep track of the number of frames since network discvery
        
        # Empty dual-hop parameters by default
        self.dhPropDelays = None
        self.dhRelays = None
        self.sfLengths = None

        # Empty variables assigned in init() but referred to in json
        self.nm = None
        self.nodeAddr = None
        self.wdt = None  # WDT passed from mainloop to be fed during longer loops.

        # Other network variables assigned later but used in json
        self.shNodes = None
        self.shPropDelays = None
        self.lq = None
        self.txDelays = None

        # Network Discovery Info
        self.last_discovery_type = None
        self.last_discovery_starttime = None
        self.last_discovery_endtime = None
        self.last_net_scheduling_starttime = None
        self.last_net_scheduling_endtime = None

           
    #########################    
    # Initialisation method #
    ######################### 
    def init(self, modem, node_addr, wdt=None):

        # This function can be called multiple times with new node addresses (node_addr can therefore change length).

        # Store a reference to the modem object the list of sensor node addresses
        self.nm = modem
        self.nodeAddr = node_addr.copy()
        self.relayLoads = [0]*len(self.nodeAddr)
        self.wdt = wdt

        # Reset some of the variables as we've changed the nodeAddr list.
        self.shNodes = None
        self.shPropDelays = None
        self.lq = None
        self.txDelays = None

        # Feed the watchdog
        if self.wdt:
            self.wdt.feed()

        # Store this node's address and check the battery voltage
        print('Initializing the gateway node...')
        retry_count = 0
        thisNode = -1
        while thisNode == -1 and retry_count < 3:
            thisNode = self.nm.get_address()
            retry_count = retry_count + 1
            utime.sleep_ms(100)

        print("  This node's address: " + '%03d' % thisNode)
        self.thisNode = thisNode

        # Feed the watchdog
        if self.wdt:
            self.wdt.feed()

        retry_count = 0
        voltage = -1
        while voltage == -1 and retry_count < 3:
            voltage = self.nm.get_battery_voltage()
            retry_count = retry_count + 1
            utime.sleep_ms(100)

        print("  Voltage supplied to the modem: " + '%0.2f' % voltage + "V")
        print("")


    ############################################
    # Method to set the link quality threshold #
    ############################################
    def set_link_quality_threshold(self, link_quality_threshold):
        if 0 < link_quality_threshold <= 5:
            self.lqThreshold = link_quality_threshold


    #######################################    
    # Method to perform network discovery #
    ####################################### 
    def do_net_discovery(self, full_rediscovery=False):

        # Feed the watchdog
        if self.wdt:
            self.wdt.feed()

        # Network Discovery Info
        if full_rediscovery:
            self.last_discovery_type = "full"
        else:
            self.last_discovery_type = "partial"

        self.last_discovery_starttime = utime.time()


        # If a full network rediscovery is required, reset the list of missing links 
        if full_rediscovery:
            self.missingLinks = None
            
        # Reset the frame counter
        self.frameCounter = 0

        # Perform single-hop network discovery, store the propagation delays and link quality
        print("*** Single-hop network discovery ***")
        numNodes = len(self.nodeAddr)
        shlq = [0]*numNodes
        nodesToTest = self.nodeAddr.copy()

        # If we've called init with a new set of node addresses then we need default shPropDelays to start with
        if not self.shPropDelays:
            self.shPropDelays = [1000000]*numNodes  # propagation delays (huge value by default)

        # If a full network rediscovery is not needed, avoid retesting very high/low quality single-hop links to save time
        if (not full_rediscovery) and self.dataPacketSR:
            for n in range(numNodes):
                if self.shNodes[n] and (self.dataPacketSR[n] >= self.srThreshold):
                    nodesToTest.remove(self.nodeAddr[n])
                    shlq[n] = self.lqThreshold
                elif self.shNodes[n] and (self.dataPacketSR[n] < self.lowSrThreshold):
                    nodesToTest.remove(self.nodeAddr[n])
        # Also avoid retesting the completely missing links
        if (not full_rediscovery) and self.missingLinks:
            for link in self.missingLinks:
                if (len(link) == 2) and (link[0] == self.thisNode) and (link[1] in nodesToTest):
                    nodesToTest.remove(link[1])
            
        # Do the single-hop network discovery for the required nodes
        (shpd, shlqNew) = gwf.doNetDiscovery(self.nm, self.thisNode, nodesToTest, self.wdt)
        for n in range(len(nodesToTest)):
            nodeInd = self.nodeAddr.index(nodesToTest[n])
            self.shPropDelays[nodeInd] = shpd[n]
            shlq[nodeInd] = shlqNew[n]
            # If any of these links failed to connect with a ping, add them to the list
            if (shlqNew[n] == 0) and (shpd[n] > 1000):
                if (not self.missingLinks):
                    self.missingLinks = list()
                self.missingLinks.append([self.thisNode, nodesToTest[n]])
            
        # Wait a guard interval before continuing
        pyb.delay(self.guardInt)
        
        # If there are no nodes connected at all, display a message and exit the function
        if all(lq == 0 for lq in shlq):
            print("  No nodes in the network!")
            self.last_discovery_endtime = utime.time()
            return False
            
        # Reset the data packet success ratio list for the next data gathering cycle
        self.dataPacketSR = None
        
        # Note all nodes that can be directly connected to the gateway based on link quality
        lqt = self.lqThreshold
        self.shNodes = [False]*numNodes
        # Select only the good links if dual-hop networking is enabled
        if self.dualHop:
            while (not any(self.shNodes)) and (lqt > 0):
                for n in range(numNodes):
                    self.shNodes[n] = (shlq[n] >= lqt)
                if not any(self.shNodes):
                    lqt -= 1 # if no nodes meet the LQ threshold, lower it
        # Select all existing links if only single-hop networking is enabled
        else:
            for n in range(numNodes):
                self.shNodes[n] = (shlq[n] > 0)
        
        # If not all nodes are connected, and if dual-hop TDA-MAC is used, continue network discovery and setup
        if (not all(self.shNodes)) and self.dualHop:

            # Feed the watchdog
            if self.wdt:
                self.wdt.feed()

            # Dual-hop network discovery stage
            print("*** Dual-hop network discovery ***")
            (self.dhPropDelays, self.dhNodes, self.dhRelays, dhlq, newMissingLinks) = \
                gwf.do2HNetDiscovery(self.nm, self.thisNode, self.nodeAddr, self.shNodes, \
                                     self.relayLoads, self.lqThreshold, self.wdt, self.missingLinks)
            # Append newly discovered missing links to the list
            if newMissingLinks:
                for link in newMissingLinks:
                    if len(link) == 2: # just to check the format
                        self.missingLinks.append(link)
                        
            pyb.delay(self.guardInt)
                                                        
            # If any dual-hop connections have a lower/same link quality than the single-hop connection, choose single-hop
            for n in range(numNodes):
                if (shlq[n] > 0) and (shlq[n] >= dhlq[n]):
                    self.shNodes[n] = True
                    self.dhNodes[n] = False
                    self.dhRelays[n] = -1
                    self.dhPropDelays[n] = 1000000
                    dhlq[n] = 0
                    
        # Create the list of directly connected node addresses
        self.shNodeAddr = list()
        for n in range(len(self.nodeAddr)):
            if self.shNodes[n]:
                self.shNodeAddr.append(self.nodeAddr[n])

        # Feed the watchdog
        if self.wdt:
            self.wdt.feed()

        # Print out the discovered network topology
        print('')
        print("*** Network Topology ***")
        self.lq = [0]*numNodes
        for n in range(len(self.nodeAddr)):
            if self.shNodes[n]:
                self.lq[n] = shlq[n]
                print("N" + "%03d" % self.nodeAddr[n] + " direct: distance - " + \
                        '%d' % round(self.shPropDelays[n]*1.5)  + 'm, LQ - ' + str(shlq[n]))
            elif self.dhNodes[n]:
                self.lq[n] = dhlq[n]
                print("N" + "%03d" % self.nodeAddr[n] + " via N" + "%03d" % self.dhRelays[n] + ": distance to relay - " + \
                        '%d' % round(self.dhPropDelays[n]*1.5)  + 'm, LQ - ' + str(dhlq[n]))
            else:
                print("N" + "%03d" % self.nodeAddr[n] + ": not connected")
        print("")

        self.last_discovery_endtime = utime.time()
        # Return True if any nodes were discovered 
        return True
    
    ###############################################################################
    # Method to set up the network schedule and distribute it to all sensor nodes #
    ###############################################################################
    def setup_net_schedule(self, guard_int=500):

        self.last_net_scheduling_starttime = utime.time()

        # Calculate transmit delays that need to be assigned to the connected nodes, and the frame length
        print("*** Network setup ***")
        self.guardInt = guard_int
        (shTxDelays, self.shFrameLength) = gwf.calcTDAMACSchedule(self.shPropDelays, self.shNodes, self.guardInt)
        pyb.delay(self.guardInt)

        # Feed the watchdog
        if self.wdt:
            self.wdt.feed()

        # Distribute transmit delay instructions to all direct nodes
        gwf.sendTDIPackets(self.nm, self.thisNode, self.nodeAddr, shTxDelays, 0, self.shNodes, self.wdt)
        
        # Save the single-hop transmit delays into a new list
        self.txDelays = shTxDelays.copy()
        
        # If some nodes are connected via dual-hop, setup dual-hop TDA-MAC
        self.relayAddr = list()
        if any(self.shNodes) and (not all(self.shNodes)) and self.dualHop:  
            
            # Make a list of relay nodes
            for n in range(len(self.nodeAddr)):
                if self.nodeAddr[n] in self.dhRelays:
                    self.relayAddr.append(self.nodeAddr[n])
            
            # Loop through every relay and calculate Tx delays and subframe lengths for dual-hop TDA-MAC
            self.sfLengths = [0]*len(self.relayAddr)
            for n in range(len(self.relayAddr)):
                
                # Make a list of propagation delays of dual-hop nodes connected via this relay
                propDelays = list()
                childNodeAddr = list()
                for k in range(len(self.nodeAddr)):
                    if self.dhRelays[k] == self.relayAddr[n]:
                        propDelays.append(self.dhPropDelays[k])
                        childNodeAddr.append(self.nodeAddr[k])
                        
                # Record the load of this relay node in this network cycle
                self.relayLoads[self.nodeAddr.index(self.relayAddr[n])] += len(childNodeAddr)

                # Feed the watchdog
                if self.wdt:
                    self.wdt.feed()

                # Calculate the TDA-MAC schedule
                (txDelays, self.sfLengths[n]) = gwf.calcTDAMACSchedule(propDelays, [True]*len(propDelays), self.guardInt)   
                # Distribute TDI packets to all dual-hop nodes
                gwf.send2HopTDIPackets(self.nm, self.thisNode, self.relayAddr[n], childNodeAddr, txDelays, self.sfLengths[n], self.wdt)
                pyb.delay(self.guardInt)
                
                # Note the transmit delay for each child node
                for a in childNodeAddr:
                    self.txDelays[self.nodeAddr.index(a)] = txDelays[childNodeAddr.index(a)]

        self.last_net_scheduling_endtime = utime.time()
        print("*** Network setup complete ***")
        
    ###############################################################    
    # Method to extract the network topology and schedule as JSON #
    ###############################################################
    def get_net_info_json(self):
        
        # Put the network connection pattern, propagation delays, link qualities, and TDA-MAC schedule into a JSON object
        # Add the Network Discovery Info too
        jason = {"addresses": self.nodeAddr,
                 "direct_connections": self.shNodes,
                 "single_hop_prop_delays": self.shPropDelays,
                 "dual_hop_prop_delays": self.dhPropDelays,
                 "relays": self.dhRelays,
                 "link_quality": self.lq,
                 "transmit_delays": self.txDelays,
                 "subframe_lengths": self.sfLengths,
                 "last_discovery_type": self.last_discovery_type,
                 "last_discovery_starttime": self.last_discovery_starttime,
                 "last_discovery_endtime": self.last_discovery_endtime,
                 "last_net_scheduling_starttime": self.last_net_scheduling_starttime,
                 "last_net_scheduling_endtime": self.last_net_scheduling_endtime
                 }
        return jason
    
    #####################################################################    
    # Method to perform a round of data gathering from all sensor nodes #
    ##################################################################### 
    def gather_sensor_data(self, time_till_next_frame, stay_awake, data_type="S"):
        
        #############################
        # Single-hop data gathering #
        #############################
        print("")
        print("*** Data gathering cycle ***")
        print("")
        frameStartTime = utime.ticks_ms()
        rxPackets = list()
        self.frameCounter += 1 
        
        # Initialise the data packet success ratio list if this is the first round in a cycle
        if not self.dataPacketSR:
            self.dataPacketSR = [1]*len(self.nodeAddr)
        
        # If necessary try to gather the data multiple times (with retransmission opportunities)
        nodesToRespond = self.shNodeAddr.copy()
        maxNumREQs = 3
        singleHopTimeout = self.shFrameLength + 2*self.guardInt # single-hop timeout = frame dur. + 2x guard interval for safety
        canGoToSleep = (not stay_awake) # allow the nodes to go to sleep between frames?
        for reqIndex in range(maxNumREQs):

            # Feed the watchdog
            if self.wdt:
                self.wdt.feed()

            # Transmit broadcast REQ
            ttnf = time_till_next_frame - utime.ticks_diff(utime.ticks_ms(), frameStartTime)
            print("Sending Broadcast REQ...")
            reqTime = utime.ticks_ms()
            gwf.sendBroadcastREQ(self.nm, data_type, reqIndex+1, ttnf, canGoToSleep, nodesToRespond)
                
            # Start a polling loop receiving the data packets
            while utime.ticks_diff(utime.ticks_ms(), utime.ticks_add(reqTime, singleHopTimeout)) < 0:

                # Feed the watchdog
                if self.wdt:
                    self.wdt.feed()

                # If there is a packet received, process it
                self.nm.poll_receiver()
                self.nm.process_incoming_buffer()
                if self.nm.has_received_packet():             
                    # Read the incoming data packet
                    packet = self.nm.get_received_packet()   
                    packetType = packet.packet_type
                    payload = bytes(packet.packet_payload)
                    if (packetType == 'U') and (payload[0:3] == b'UND'):
                        # Store the packet and decode the packet source
                        rxPackets.append(packet)
                        src = struct.unpack('B', payload[3:4])[0]     
                        print("  Packet received from N" + "%03d" % src + ": " + str(payload))  
                        # Update the packet success ratio from this node
                        nodeInd = self.nodeAddr.index(src)
                        self.dataPacketSR[nodeInd] = ((self.frameCounter-1)/self.frameCounter) * self.dataPacketSR[nodeInd] \
                                                         + (1/self.frameCounter) * 1 / (reqIndex+1)
                        # Remove this node from the list of nodes expected to respond
                        if src in nodesToRespond:
                            nodesToRespond.remove(src)
                        
                # Add a delay before checking the serial port again     
                pyb.delay(25)
                
            # If there are more nodes that need to respond, move on
            if not nodesToRespond:
                break
         
        # If there are any nodes still left to respond, update their packet success ratio
        for n in nodesToRespond:
            nodeInd = self.nodeAddr.index(n)
            self.dataPacketSR[nodeInd] = ((self.frameCounter-1)/self.frameCounter) * self.dataPacketSR[nodeInd] # + 0
         
        ###########################
        # Dual-hop data gathering #
        ###########################
        # Loop through each relay node and gather the data from its branch    
        for r in self.relayAddr:

            # Establish the set of nodes that are expected to respond (the child nodes for this relay)
            propDelay = self.shPropDelays[self.nodeAddr.index(r)]
            nodesToRespond = list()
            for k in range(len(self.nodeAddr)):
                if self.dhRelays[k] == r:
                    nodesToRespond.append(self.nodeAddr[k])
                    
            # Request data from the relay node multiple times if needed (to allow retransmissions)
            sdtInitiated = False
            sdtTimeout = 60000 # minute total timeout for the relay to initiate data transfer
            noResponseFromRelay = False
            for reqIndex in range(maxNumREQs):

                # Feed the watchdog
                if self.wdt:
                    self.wdt.feed()

                # Transmit a unicast REQ to the relay node
                print("Sending Unicast REQ to N" + "%03d" % r + "...")
                reqTime = utime.ticks_ms()
                try:
                    ureqAcked = gwf.sendUnicastREQ(self.nm, data_type, reqIndex+1, self.thisNode, r, canGoToSleep, nodesToRespond, self.wdt)
                except Exception as e:
                    print("Error on l. 383 in gw_node.py: " + str(e))
                
                # Once the SDT handshake was received, the packets should follow in a "train"          
                dtTimeout= 2*propDelay + 3*self.guardInt + len(nodesToRespond)*(gwf.dataPktDur + self.guardInt)
                timeoutReached = False
                # If this is a retransmission REQ, reset the data transfer time window
                if reqIndex > 0:
                    sdtTime = utime.ticks_ms()
                # Enter a loop listening for data packets
                while ureqAcked and nodesToRespond and (not timeoutReached):

                    # Feed the watchdog
                    if self.wdt:
                        self.wdt.feed()

                    # If there is a packet received, process it
                    self.nm.poll_receiver()
                    self.nm.process_incoming_buffer()
                    if self.nm.has_received_packet():             
                        # Read the incoming packet
                        packet = self.nm.get_received_packet()   
                        packetType = packet.packet_type
                        payload = bytes(packet.packet_payload)
                        # SDT handshake packet
                        if (packetType == 'U') and (payload[0:5] == b'UNSDT'):
                            print("  Data transfer initiated by the relay...")
                            sdtInitiated = True
                            sdtTime = utime.ticks_ms() 
                        # Data packet
                        elif (packetType == 'U') and (payload[0:3] == b'UND'):
                            # Process the packet
                            # Store the packet and decode the packet source
                            rxPackets.append(packet)
                            src = struct.unpack('B', payload[3:4])[0]     
                            print("  Packet received from N" + "%03d" % src + ": " + str(payload))               
                            # Remove this node from the list of nodes expected to respond
                            if src in nodesToRespond:
                                nodesToRespond.remove(src)
                            
                    # Check if the timeout is reached
                    if (not sdtInitiated) and utime.ticks_diff(utime.ticks_ms(), utime.ticks_add(reqTime, sdtTimeout)) > 0:
                        noResponseFromRelay = True
                        timeoutReached = True
                    if sdtInitiated and utime.ticks_diff(utime.ticks_ms(), utime.ticks_add(sdtTime, dtTimeout)) > 0:
                        timeoutReached = True
                            
                    # Add a delay before checking the serial port again     
                    pyb.delay(25)
                            
                # If there is no response from relay after 60 sec, or all data has been gathered, move on
                if noResponseFromRelay or (not nodesToRespond):
                    break  
                    
        print("")
        print("*** Data gathering cycle completed ***")
        print("")
        
        # Return the list of packets received in this cycle
        return rxPackets
        
    # Method to instruct the network to go to sleep for a specified time
    def set_network_to_sleep(self, time_till_next_tx):
        
        print("*** Sending the network to sleep for " + str(time_till_next_tx) + " msec ***")
        initTime = utime.ticks_ms()
        
        # Send multiple copies of blank broadcast REQs (fopr redundancy)
        numREQCopies = 3
        interval = 2000 # 2 second intervals between REQ copies
        for n in range(numREQCopies):

            # Feed the watchdog
            if self.wdt:
                self.wdt.feed()

            # Transmit blank broadcast REQ
            ttnf = time_till_next_tx - utime.ticks_diff(utime.ticks_ms(), initTime)
            print("Sending Blank Broadcast REQ...")
            gwf.sendBroadcastREQ(self.nm, "S", n+1, ttnf, True, [])
            
            # Wait for a set interval before transmitting it again
            pyb.delay(interval)
            
        # Loop through each relay node and send a blacnk unicast REQ to them (to reach their child nodes) 
        interval = 7000
        for r in self.relayAddr:

            # Feed the watchdog
            if self.wdt:
                self.wdt.feed()

            print("Sending Blank Unicast REQ to N" + "%03d" % r)
            gwf.sendUnicastREQ(self.nm, "S", 1, self.thisNode, r, True, [], self.wdt)
            pyb.delay(interval)
