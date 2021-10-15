#! /usr/bin/env python
#
# MicroPython TDA-MAC network protocol functions related to getaway/relay node functionality
# This file is part of the following repository:
# https://github.com/bensherlock/micropython-usmart-network
#
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
from uac_modem.main.unm3driver import MessagePacket, Nm3

# Static packet duration parameters [msec]
dataPktDur = round(105 + (64+16) * 12.5) # max 64-byte data packet duration
reqPktDur = round(105 + (64+16) * 12.5) # REQ packet duration (max 64 bytes)
tdiPktDur = round(105 + (14+16) * 12.5) # TDI packet duration
ackPktDur = round(105 + (5+16) * 12.5) # ACK packet
defGuardInt = 250 # default 250 ms guard interval for network discovery and setup transmissions

### Function that performs initial network discovery
def doNetDiscovery(nm, thisNode, nodeAddr, wdt=None):

    # Function parameters
    numTestTx = 5    # Number of test transmissions for each node
    numPingAttempts = 3 # number of initial ping exchange attempts to establish presence of link
    timeout = 3.0 # 3 ping second timeout
    
    # Define the test transmission payload
    testMSG = b'UNNDTX' + struct.pack('B', int(thisNode)) + b'USMART_Test_Transmission'

    # Loop through every node, send multiple test messages to it, and note the propagation delays
    propDelays = [1000000]*len(nodeAddr) # propagation delays (huge value by default)
    linkQuality = [0]*len(nodeAddr) # link quality is represented by the number of successful test message exchanges
    for n in range(len(nodeAddr)):
    
        # Try pinging this node to measure the propagation delay
        for k in range(numPingAttempts):

            # Feed the watchdog
            if wdt:
                wdt.feed()
            
            # Send a ping and wait for a repsponse  
            print("Pinging N" + "%03d" % nodeAddr[n] + "...")          
            delay = nm.send_ping(nodeAddr[n], timeout)
            pingSuccess = (delay > 0)
            # If a response was received, note the propagation delay and move on to link tests
            if pingSuccess:
                propDelays[n] = round(delay*1e3) # msec
                print("  Ping response received: " + '%.3f' % delay + " sec delay")
                pyb.delay(defGuardInt) # add guard interval before next transmission
                break                
            else:
                print("  No ping response")
         
        # Send multiple test messages to the node and measure the link quality
        if pingSuccess:
            for k in range(numTestTx):

                # Feed the watchdog
                if wdt:
                    wdt.feed()
                    
                # Set the timeout for test transmissions based on the measured propagation delay
                thisNodeTimeout = 2 * (propDelays[n] + dataPktDur + defGuardInt)

                # Send the test message to the node
                print("Sending test MSG to N" + "%03d" % nodeAddr[n] + "...")
                nm.send_unicast_message(nodeAddr[n], testMSG) 
                
                # Wait for the test message response
                timerStart = utime.ticks_ms()
                while utime.ticks_diff(utime.ticks_ms(), utime.ticks_add(timerStart, thisNodeTimeout)) < 0:

                    # Feed the watchdog
                    if wdt:
                        wdt.feed()

                    # Check if a packet has been received
                    nm.poll_receiver()
                    nm.process_incoming_buffer()
                    if nm.has_received_packet():        
                        # Read the incoming packet
                        packet = nm.get_received_packet()
                        payload = bytes(packet.packet_payload)
                        payloadLength = len(payload)
                        
                        # If the test message response is received, increment the link quality and move on
                        if (payloadLength > 7) and (payload[0:6] == b'UNNDRX') and (struct.unpack('B', payload[6:7])[0] == nodeAddr[n]):
                            print("  Test message response received")
                            linkQuality[n] += 1
                            pyb.delay(defGuardInt) # add guar interval before next transmission
                            break
                        else:
                            print("  Packet received but not recognised as the test message response")
                
            # Print the link quality test result
            print("Link quality with N" + "%03d" % nodeAddr[n] + ": " + str(linkQuality[n]) + "/" + str(numTestTx))
                
    # Return the list of propagation delays
    return propDelays, linkQuality


### Function that performs dual-hop network discovery after direct nodes have been found
def do2HNetDiscovery(nm, thisNode, nodeAddr, directNodes, relayLoads, lqThreshold, wdt=None, skipLinks=None):
    
    # Initialize the lists of dual-hop nodes, their propagation delays and relays, and a list to keep track of missing links
    dhNodes = [False]*len(nodeAddr)
    dhPropDelays = [1e9]*len(nodeAddr)
    dhRelays = [-1]*len(nodeAddr)
    dhLQ = [0]*len(nodeAddr)
    missingLinks = list()
    
    # Create a list of nodes that are not directly connected to gateway
    uncNodeAddr = list()
    for n in range(len(nodeAddr)):
        if not directNodes[n]:
            uncNodeAddr.append(nodeAddr[n])
            
    # Loop through the directly connected nodes and tell them to perform network discovery
    # Sort the order of relay selection to spread the load equally over many cycles
    nodeList = sorted(range(len(nodeAddr)), key=lambda k: relayLoads[k])
    for n in nodeList:
        if directNodes[n]:
        
            # Limit the number of potential leaf nodes to 10 (just in case!)
            maxNumLeafNodes = 10
            thisUncNodeSet = uncNodeAddr.copy()
            if len(thisUncNodeSet) > maxNumLeafNodes:
                thisUncNodeSet = thisUncNodeSet[0:maxNumLeafNodes]
                
            # If any of these links need to be skipped, do not test them
            if skipLinks:
                for link in skipLinks:
                    if (len(link) == 2) and (link[0] == nodeAddr[n]) and (link[1] in thisUncNodeSet):
                        thisUncNodeSet.remove(link[1])
                        
            # If there are any nodes in the target set still, instruct the relay to perform network discovery
            if thisUncNodeSet:
            
                # Create the network discovery request packet with addresses of unconnected nodes
                reqPacket = b'UNN?' + struct.pack('B', int(thisNode))
                for addr in thisUncNodeSet:
                    reqPacket += struct.pack('B', int(addr))
                
                # Try sending the packet multiple times to make sure it is received
                maxTries = 5
                reqReceived = False
                for k in range(maxTries):

                    # Feed the watchdog
                    if wdt:
                        wdt.feed()
            
                    # Send the packet
                    print("Sending the network discovery request to N" + "%03d" % nodeAddr[n])
                    response = nm.send_unicast_message_with_ack(nodeAddr[n], reqPacket)
                            
                    # Check if the ACK was received
                    if response > 0:                   
                        # Success, move on
                        print("  ACK received")
                        reqReceived = True
                        break
                    else:
                        print("  No ACK")
                            
                # If the request was acknowledged, wait for the network discovery results (2 min timeout)
                if reqReceived:
                    netDiscTimeout = 240000
                    timerStart = utime.ticks_ms()
                    while utime.ticks_diff(utime.ticks_ms(), utime.ticks_add(timerStart, netDiscTimeout)) < 0:

                        # Feed the watchdog
                        if wdt:
                            wdt.feed()

                        # Check if a packet has been received
                        nm.poll_receiver()
                        nm.process_incoming_buffer()
                        if nm.has_received_packet(): 
        
                            # Read the incoming packet                  
                            packet = nm.get_received_packet()   
                            packetType = packet.packet_type
                            payload = bytes(packet.packet_payload)
                        
                            # If this is a node discovery response, parse it
                            if (packetType[0:1] == 'U') and (len(payload) > 4) and (payload[0:4] == b'UNNR'):
                                
                                # Loop through every 5 bytes and parse it as <int><short> = <prop. delay><link quality>
                                propDelays = list()
                                for k in range(len(thisUncNodeSet)):
                                    # Decode the propagation delay
                                    intBytes = payload[4+k*5 : 4+(k+1)*5-1]
                                    propDelay = struct.unpack('I', intBytes)[0]
                                    # If the propagation delay failed to be measured (no ping), add this as a missing link
                                    if propDelay > 100000:
                                        missingLinks.append([nodeAddr[n], thisUncNodeSet[k]])
                                    # Decode the link quality
                                    lq = struct.unpack('B', payload[4+k*5+4 : 4+(k+1)*5])[0]
                                    # If the link quality exceeds the current best link quality, update the route
                                    nodeIndex = nodeAddr.index(thisUncNodeSet[k])
                                    if lq > dhLQ[nodeIndex]:                           
                                        dhNodes[nodeIndex] = True
                                        dhPropDelays[nodeIndex] = propDelay
                                        dhRelays[nodeIndex] = nodeAddr[n]
                                        dhLQ[nodeIndex] = lq
                                        
                                # Print message and log it
                                print("  Node discovery results received from N" + "%03d" % nodeAddr[n]) 
                                pyb.delay(defGuardInt)
                                break               
        
            # Update the list of unconnected nodes (with the link quality below threshold)
            uncNodeAddr = list()
            for k in range(len(nodeAddr)):
                if (not directNodes[k]) and (dhLQ[k] < lqThreshold):
                    uncNodeAddr.append(nodeAddr[k])
                    
            # If all nodes are connected, finish the network discovery
            if not uncNodeAddr:
                print("All nodes connected with link quality above threshold (" + str(lqThreshold) + ")!")
                break
            
    # Return the dual-hop topology parameters
    return dhPropDelays, dhNodes, dhRelays, dhLQ, missingLinks
    
### Function to send TDI packets to all connected nodes, return updated list of connected nodes
def sendTDIPackets(nm, thisNode, nodeAddr, txDelays, sfLength, connNodes, wdt=None):
    
    maxTries = 3    # maximum number of tries to receive ACK from node
    timeout = 5000  # ACK timeout 5 seconds 
    deliverySuccess = [False]*len(nodeAddr)
    
    # Loop through every connected node and send a TDI packet with ACK to it
    for n in range(len(nodeAddr)):
        if connNodes[n]:
                
            # Try sending the TDI packet multiple times if needed
            for k in range(maxTries):

                # Feed the watchdog
                if wdt:
                    wdt.feed()
                
                # Send unicast TDI packet that requires an ACK
                print("Sending TDI to N" + "%03d" % nodeAddr[n] + "...")
                # Transmit the packet
                tdiPacket = b'UNI!' + struct.pack('B', thisNode) + struct.pack('B', nodeAddr[n]) + \
                            struct.pack('I', txDelays[n]) + struct.pack('I', sfLength)
                nm.send_unicast_message(nodeAddr[n], tdiPacket)
                pyb.delay(tdiPktDur) # sleep for the duration of transmission
            
                # Wait for the TDI ACK
                timerStart = utime.ticks_ms()
                ackReceived = False
                while utime.ticks_diff(utime.ticks_ms(), utime.ticks_add(timerStart, timeout)) < 0:

                    # Feed the watchdog
                    if wdt:
                        wdt.feed()

                    # Check if a packet has been received
                    nm.poll_receiver()
                    nm.process_incoming_buffer()
                    if nm.has_received_packet():        
                        # Read the incoming packet
                        packet = nm.get_received_packet()
                        payload = bytes(packet.packet_payload)
                        payloadLength = len(payload)
                        
                        # If this is a unicast packet, check that it is the ACK from the sensor node
                        if (payloadLength > 4) and (payload[0:4] == b'UNIA') and (struct.unpack('B', payload[4:5])[0] == nodeAddr[n]):
                            print("  TDI ACK received")
                            deliverySuccess[n] = True
                            ackReceived = True
                            break
                        else:
                            print("  Packet received but not recognised as TDI ACK")
                            
                # If the ACK was received, move on to next node
                if ackReceived:
                    pyb.delay(defGuardInt)
                    break
                    
    # Return the delivery success
    return deliverySuccess
    
### Function to send TDI packets to dual-hop nodes of a given relay branch
def send2HopTDIPackets(nm, thisNode, relayAddr, nodeAddr, txDelays, sfLength, wdt=None):
    
    maxTries = 5        # maximum number of tries to receive ACK from the node
    timeout1Hop = 5.0   # single hop timeout 5 seconds  
    
    # Loop through every dual hop node and send a TDI to it via its relay
    for n in range(len(nodeAddr)):
            
        # Try sending the TDI packet multiple times if needed
        for k in range(maxTries):

            # Feed the watchdog
            if wdt:
                wdt.feed()

            # Send unicast TDI packet that requires an ACK
            print("Sending TDI to N" + "%03d" % nodeAddr[n] + " via N" + "%03d" % relayAddr + "...")
            
            # Transmit the packet
            tdiPacket = b'UNI!' + struct.pack('B', int(thisNode)) + struct.pack('B', int(nodeAddr[n])) \
                                + struct.pack('I', txDelays[n]) + struct.pack('I', sfLength)
            response = nm.send_unicast_message_with_ack(relayAddr, tdiPacket, timeout1Hop)
        
            # Check if the ACK was received
            if response > 0:                    
                print("  ACK received from relay")
                pyb.delay(defGuardInt)
                break # success, move on
            else:
                print("  No ACK") # try again
                    
        # Wait for the end-to-end TDI ACK
        timeout2Hops = 40000 # dual hop timeout 40 seconds
        timerStart = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), utime.ticks_add(timerStart, timeout2Hops)) < 0:

            # Feed the watchdog
            if wdt:
                wdt.feed()

            # Check if a packet has been received
            nm.poll_receiver()
            nm.process_incoming_buffer()
            if nm.has_received_packet():
                # Read the incoming packet
                packet = nm.get_received_packet()
                payload = bytes(packet.packet_payload)
                
                # If this is a TDI ACK/NACK from the relay
                if (len(payload) == 5) and (payload[0:3] == b'UNI') and (struct.unpack('B', payload[4:5])[0] == relayAddr):
                    # Print a message
                    if payload[3:4] == b'A':
                        print("  Dual-hop TDI ACK received")
                    elif payload[3:4] == b'N':
                        print("  Dual-hop TDI NACK received")
                    pyb.delay(defGuardInt)
                    break
                    
### Function to transmit a single broadcast REQ packet
def sendBroadcastREQ(nm, dataType, reqIndex, timeTillNextFrame, canGoToSleep, nodeAddr):
    
    # Generate the REQ packet bytes
    sleepFlag = struct.pack('B', 1) if (canGoToSleep) else struct.pack('B', 0)
    ttnf = struct.pack('I', timeTillNextFrame)
    dt = b'L' if (dataType == "L") else b'S'
    reqPacket = b'UNR' + struct.pack('B', reqIndex) + ttnf + sleepFlag + dt
            
    # Append the specified addresses to the end
    for addr in nodeAddr:
        reqPacket += struct.pack('B', addr)
        
    # Transmit the broadcast REQ packet
    nm.send_broadcast_message(reqPacket)
    pyb.delay(reqPktDur) 
    
### Function to transmit a single Unicast REQ packet
def sendUnicastREQ(nm, dataType, reqIndex, thisNode, destNode, canGoToSleep, nodeAddr, wdt=None):
    
    # Generate the REQ packet bytes
    sleepFlag = struct.pack('B', 1) if (canGoToSleep) else struct.pack('B', 0)
    dt = b'L' if (dataType == "L") else b'S'
    reqPacket = b'UNR' + struct.pack('B', thisNode) + struct.pack('B', reqIndex) + sleepFlag + dt
    # Append the specified addresses to the end
    for addr in nodeAddr:
        reqPacket += struct.pack('B', addr)
    
    # Transmit the unicast REQ packet with an initial ACK from the relay node
    numTries = 3
    timeout = 5.0
    for n in range(numTries):

        # Feed the watchdog
        if wdt:
            wdt.feed()

        delay = nm.send_unicast_message_with_ack(destNode, reqPacket, timeout)
        if delay > 0:
            break
            
    # Return a binary flag indicating whether an ACK was received
    return (delay > 0)

### Function to calculate TDA-MAC transmit delays
def calcTDAMACSchedule(propDelays, connNodes, guardInt):
    
    # Set the minimum delay to allow sensor read / packet processing delay
    minDelay = 200 # msec
    
    # Sort the propagation delays from shortest to longest
    numNodes = len(propDelays)
    for n in range(numNodes):
        if not connNodes[n]:
            propDelays[n] = 1000000 # set prop delays of unconnected nodes to a large value
    sInd = sorted(range(numNodes), key = lambda k: propDelays[k])
    
    # Loop through all nodes and calculate their transmit delays (0 by default)
    txDelays = [0]*numNodes
    for n in range(numNodes):
        
        # Delay of the first node is at the minimum value
        if (n==0):
            txDelays[sInd[n]] = minDelay
        
        # Calculate transmit delays for all other nodes
        if (n > 0) and connNodes[sInd[n]]:
            
            # Calculate the transmit delay
            txDelays[sInd[n]] = txDelays[sInd[n-1]] + dataPktDur + guardInt - 2*(propDelays[sInd[n]] - propDelays[sInd[n-1]])
            # Make sure it is not shorter than the minimum
            txDelays[sInd[n]] = max(txDelays[sInd[n]], minDelay)
    
    # Calculate the overall frame length of the TDA-MAC scchedule
    frameLength = max([2*propDelays[n] + txDelays[n] for n in range(numNodes) if connNodes[n]]) + dataPktDur + reqPktDur + guardInt
            
    # Return the transmit delay list and the frame length
    return txDelays, frameLength
