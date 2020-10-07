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

# Static packet duration parameters
dataPktDur = 0.105 + (64+16) * 0.0125 # max 64-byte data packet duration
reqPktDur = 0.105 + (64+16) * 0.0125 # REQ packet duration (max 64 bytes)
tdiPktDur = 0.105 + (18+16) * 0.0125 # 8-byte TDI packet duration
ackPktDur = 0.105 + (5+16) * 0.0125 # 5-byte ACK packet

### Function that performs initial network discovery
def doNetDiscovery(nm, nodeAddr):

    # Function parameters
    numTestTx = 5 # Number of test transmissions for each node
    timeout = 5.0 # 5 second timeout
    testMSG = "USMART Network Discovery"

    # Loop through every node, send multiple test messages to it, and note the propagation delays
    propDelays = [1e9]*len(nodeAddr) # propagation delays (huge value by default)
    linkQuality = [0]*len(nodeAddr) # link quality is represented by the number of successful test message exchanges
    for n in range(len(nodeAddr)):
         
        # Send a test message to the node and parse the response
        for k in range(numTestTx):
            
            # Ping the node
            print("Sending test MSG to N" + "%03d" % nodeAddr[n] + "...")
            delay = nm.send_unicast_message_with_ack(nodeAddr[n], testMSG.encode('UTF-8'), timeout) 
            
            # If a response was received, store it, increment the link quality and log it
            if delay > 0:
                # Save the propagation delay and increment the link quality
                propDelays[n] = delay
                linkQuality[n] += 1
                # Print a message
                print("  Response received: " + '%.3f' % delay + "s delay")             
            # Otherwise, if there was no response, display a message
            else:
                print("  No response received")
                
    # Return the list of propagation delays
    return propDelays, linkQuality
    
### Function to send TDI packets to all connected nodes, return updated list of connected nodes
def sendTDIPackets(nm, thisNode, nodeAddr, txDelays, frameInt, sfLength, connNodes):
    
    maxTries = 3    # maximum number of tries to receive ACK from node
    timeout = 5000  # ACK timeout 5 seconds 
    
    # Loop through every connected node and send a TDI packet with ACK to it
    for n in range(len(nodeAddr)):
        if connNodes[n]:
                
            # Try sending the TDI packet multiple times if needed
            for k in range(maxTries):
                
                # Send unicast TDI packet that requires an ACK
                print("Sending TDI to node " + str(nodeAddr[n]) + "...")
                # Transmit the packet
                tdiPacket = b'UNI!' + struct.pack('B', thisNode) + struct.pack('B', nodeAddr[n]) + \
                            struct.pack('f', txDelays[n]) + struct.pack('f', frameInt) + struct.pack('f', sfLength)
                nm.send_unicast_message(nodeAddr[n], tdiPacket)
                pyb.delay(round(tdiPktDur*1000)) # sleep for the duration of transmission
            
                # Wait for the response on the serial port
                waitForPacket(nm, timeout)
                
                # If a packet was received
                if nm.has_received_packet():        
                    # Read the incoming packet
                    packet = nm.get_received_packet()
                    
                    # If this is a unicast packet, check that it is the ACK from the sensor node
                    if packet.packet_type[0:1] == 'U':
                        payload = bytes(packet.packet_payload)
                        payloadLength = len(payload)
                        # If there was a response, move on, do not try again
                        if (payloadLength > 4) and (payload[0:4] == b'UNIA') and (struct.unpack('B', payload[4:5])[0] == nodeAddr[n]):
                            print("  TDI ACK received")
                            # Move on to next node
                            break
                        else:
                            print("  Packet received but not recognised as TDI ACK")
                            
                # If there was a timeout on the last attempt, mark this node as unconnected
                elif (k == maxTries-1):
                    connNodes[n] = False
                    
    # Return updated list of connected nodes
    return connNodes
                    
### Function to transmit a single broadcast REQ packet
def sendBroadcastREQ(nm, dataType, reqIndex, timeSinceFrameStart, canGoToSleep, nodeAddr):
    
    # Generate the REQ packet bytes
    sleepFlag = struct.pack('B', 1) if (canGoToSleep) else struct.pack('B', 0)
    tsfs = struct.pack('f', timeSinceFrameStart)
    dt = b'L' if (dataType == "L") else b'S'
    reqPacket = b'UNR' + struct.pack('B', reqIndex) + tsfs + sleepFlag + dt
            
    # Append the specified addresses to the end
    for addr in nodeAddr:
        reqPacket += struct.pack('B', addr)
        
    # Transmit the broadcast REQ packet
    print("Trasmitting Broadcast REQ...")
    nm.send_broadcast_message(reqPacket)    

### Function to calculate TDA-MAC transmit delays
def calcTDAMACSchedule(propDelays, connNodes, guardInt):
    
    # Set the minimum delay to allow sensor read / packet processing delay
    minDelay = 0.2
    
    # Sort the propagation delays from shortest to longest
    numNodes = len(propDelays)
    sInd = sorted(range(numNodes), key = lambda k: propDelays[k])
    
    # Loop through all nodes and calculate their transmit delays (0 by default)
    txDelays = [0.0]*numNodes
    for n in range(numNodes):
        
        # Delay of the first node is at the minimum value
        if (n==0):
            txDelays[sInd[n]] = minDelay
        
        # Leave transmit delay of first node and all unconnected nodes at zero
        if (n > 0) and connNodes[sInd[n]]:
            
            # Calculate the transmit delay
            txDelays[sInd[n]] = txDelays[sInd[n-1]] + dataPktDur + guardInt - 2*(propDelays[sInd[n]] - propDelays[sInd[n-1]])
            # Make sure it is not shorter than the minimum
            txDelays[sInd[n]] = max(txDelays[sInd[n]], minDelay)
    
    # Calculate the overall frame length of the TDA-MAC scchedule
    frameLength = max([2.0*propDelays[n] + txDelays[n] for n in range(numNodes) if propDelays[n] < 1e8]) + dataPktDur + reqPktDur + guardInt
            
    # Return the transmit delay list
    return txDelays, frameLength

# Poll receiver in a loop for a specified length of time (milliseconds)
def waitForPacket(nm, timeout = 10*1000, step = 50):

    nm.poll_receiver()
    nm.process_incoming_buffer()
    timeoutCounter = 0
    while not nm.has_received_packet():
        pyb.delay(step)
        timeoutCounter += step
        if timeoutCounter > timeout:
            break
        nm.poll_receiver()
        nm.process_incoming_buffer()