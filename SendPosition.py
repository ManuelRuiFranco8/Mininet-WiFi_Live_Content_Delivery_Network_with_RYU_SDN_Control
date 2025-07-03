#!/usr/bin/python3

#################################################################################################################Imports
from scapy.layers.l2 import Ether #manage L2 Ethernet frames
from scapy.layers.inet import IP, UDP #manage L3 IPv4 packets and L4 UDP packets
from scapy.packet import Raw #create data payload from crafted packets
from scapy.sendrecv import sendp #send packets on L2 interfaces

import argparse #parsing CLI arguments
import logging #regulate verbosity of Python runtime output

######################################################################################Elephant Flows Detection Algorithm
logging.getLogger("scapy.runtime").setLevel(logging.ERROR)

'''Parse command-line arguments and kickstart on Mininet-emulated servers a process monitoring servers' L2 exit interface (using Scapy sniff() function) to 
   individuate transmitted elephant flows (which surpass a duration threshold and a raw data size threshold)'''
if __name__ == '__main__':
    parser=argparse.ArgumentParser(description='Monitors traffic between servers (host devices) emulated by Mininet') #argument parser for CLI parameters
    parser.add_argument('--drone', type=str, help='Name of this drone')
    parser.add_argument('--src', type=str, help='IPv4 address of this drone')
    parser.add_argument('--dst', type=str, help='IPv4 destination address')
    parser.add_argument('--pos', type=str, help='Current position of this drone')
    parser.add_argument('--occ', type=float, default=0.0, help='Current bandwidth occupation on AP')
    args=parser.parse_args() #parsed arguments

    out_intf=f'{args.drone}-wlan0' #L2 interface on server
    DSCP_MARK=0b000011<<2 #DS (or TOS equivalently) header field 00001100 (used to mark notification packets)

    load=f'pos:{args.pos}/occ:{args.occ}'

    flag_packet=Ether(dst='ff:ff:ff:ff:ff:ff')/IP(src=args.src, dst=args.dst)/UDP()/Raw(load=load)
    flag_packet[IP].tos=DSCP_MARK #mark signal packet by setting value of IPv4 DS field (DS field is equivalent to TOS field)

    for i in range(5): #send the notification 5 times to deal with losses due to wireless medium
        sendp(flag_packet, iface=out_intf) #transmit marked packet on drone's L2 exit interface

    print(f"Drone {args.drone}:{out_intf} ({args.src}). Current position: {args.pos}. Bandwidth occupation: {args.occ}\n")
    print(f"Packet: {flag_packet.show()}\n")
    print(f"Payload: {flag_packet[Raw].load}\n")
    print(f"\n")