#!/usr/bin/env python3

"""Usage:
   ryu-manager DroneController.py"""

#################################################################################################################Imports
from ryu.base import app_manager
from ryu.ofproto import ofproto_v1_3

from ryu.controller.handler import set_ev_cls
from ryu.controller.handler import MAIN_DISPATCHER, CONFIG_DISPATCHER
from ryu.controller import ofp_event
from ryu.controller.dpset import EventDP
from ryu.topology.event import EventHostAdd

from ryu.lib.packet import packet, ethernet, ether_types, ipv4, arp, udp

from datetime import datetime as dt #date-related operations
import time
import threading
import socket
import json

########################################################################################################Global Variables
log_path="/home/francesco2/Documenti/PycharmProjects/Smart2/logFiles/ControllerLog.txt" #file where controller saves logs of all performed operations
mob_log="/home/francesco2/Documenti/PycharmProjects/Smart2/logFiles/MobilityLog.txt" #file where controller saves logs related to mobility
band_log="/home/francesco2/Documenti/PycharmProjects/Smart2/logFiles/BandwidthLog.txt" #file where controller saves logs related to mobility

class FleetController(app_manager.RyuApp): #######################################################Controller Application
    OFP_VERSIONS=[ofproto_v1_3.OFP_VERSION]

    '''Creates instance of FleetController application, initialize instance attributes
       @param *args positional arguments (passed by Ryu.app_manager)
       @param **kwargs keywords arguments (passed by Ryu.app_manager)'''
    def __init__(self, *args, **kwargs):
        with open(log_path, "w") as log_file:
            log_file.write(f"{dt.now()} -> Initialize controller\n")
        with open(band_log, "w") as band_file:
            band_file.write(f"{dt.now()} -> Bandwidth Measurement Started\n")
            band_file.write(f"\n")

        super(FleetController, self).__init__(*args, **kwargs)

        self.__access_points={} #dictionary <AP name, <port_name, port_number>>
        self.__datapaths={} #dictionary <DPID, datapath>
        self.__dpids = { 13:'ap1',
                         14:'ap2',
                         15:'ap3',
                         16:'ap4'} #dictionary <DPID, AP name>

        self.__drone_positions={} #dictionary <drone name, (x, y, z)> (associating drone names to current drone positions)
        self.__drone_energy={} #dictionary <drone name, float> (associating a drone to its residual energy level)
        self.__num_endpoints=0 #counter of connected endpoints

        self.__hover_positions = { 'drone1': (26.25, 17, 35),
                                   'drone2': (78.75, 17, 35),
                                   'drone3': (78.75, 51, 35),
                                   'drone4': (26.25, 51, 35),
                                   'drone5': (26.25, 17, 35), #same as drone1
                                   'drone6': (78.75, 17, 35), #same as drone2
                                   'drone7': (78.75, 51, 35), #same as drone3
                                   'drone8': (26.25, 51, 35), #same as drone4
                                } #dictionary associating each drone of the fleet to its designated hovering position

        self.__drone_macs = { '02:00:00:00:00:00':'drone1',
                              '02:00:00:00:01:00':'drone2',
                              '02:00:00:00:02:00':'drone3',
                              '02:00:00:00:03:00':'drone4',
                              '02:00:00:00:04:00':'drone5',
                              '02:00:00:00:05:00':'drone6',
                              '02:00:00:00:06:00':'drone7',
                              '02:00:00:00:07:00':'drone8' } #dictionary <MAC address, drone name>

        self.__endpoint_macs = {'00:00:00:00:00:09': 'endpoint1',
                                '00:00:00:00:00:0a': 'endpoint2',
                                '00:00:00:00:00:0b': 'endpoint3',
                                '00:00:00:00:00:0c': 'endpoint4' } #dictionary <MAC address, endpoint name>

        self.__broadcastAddress='192.168.1.255' #network's broadcast address
        self.__base=(0.0, 0.0, 0.0) #coordinates of drones supply station
        self.__address='localhost' #address of energy management server socket
        self.__port=8080 #port number of energy management server socket

        self.__ip_groups={ 'ap1': ['192.168.1.1', '192.168.1.5', '192.168.1.9'], #IPv4 of host devices (drones/endpoints) directly connected to AP1
                           'ap2': ['192.168.1.2', '192.168.1.6', '192.168.1.10'], #IPv4 of all host devices (drones/endpoints) directly connected to AP2
                           'ap3': ['192.168.1.3', '192.168.1.7', '192.168.1.11'], #IPv4 of all host devices (drones/endpoints) directly connected to AP3
                           'ap4': ['192.168.1.4', '192.168.1.8', '192.168.1.12'] }  #IPv4 of all host devices (drones/endpoints) directly connected to AP4

        self.__spurious_ip=['192.168.1.21', '192.168.1.22'] #these IPv4s refers to trouble-maker nodes (which will connect to AP1)]

        self.__quality={ 'high' : [720, 1280, 60, 0.5], #num X pixels, num Y pixels, fps, compression rate (H.264)
                         'low' : [480, 720, 30, 0.5] } #dictionary <str, float[]> defining streams quality parameters

        self.__occupation = { 'ap1' : 0.0,
                              'ap2' : 0.0,
                              'ap3' : 0.0,
                              'ap4' : 0.0 } #dictionary <str, float> associating each AP to its current channel bandwidth occupation (percentage)

        self.__congested = { 'ap1' : 0,
                             'ap2' : 0,
                             'ap3' : 0,
                             'ap4' : 0 } #dictionary <str, int> associating each AP to a indication of wireless channel status:
                                         #0 = idle channel, 3,2 = occupied channel (do not change rules), 1 = occupied channel (change rules)

        try: #starting thread
            t=threading.Thread(target=self.energy_model,args=()) #thread handling drone energy consumption during flight and hovering
            t.start()
        except Exception as e:
            print(e)

        with open(log_path, "a") as log_file:
            log_file.write(f"{dt.now()} -> Network broadcast address: {self.__broadcastAddress}\n")
            log_file.write(f"{dt.now()} -> Controller initialized\n")
            log_file.write(f"\n")

    ####################################################################################################Thread Function
    '''Thread function computing drones energy consumption models 
       @param FleetController object'''
    def energy_model(self):

        status = { 'drone1':'Base',
                   'drone2':'Base',
                   'drone3':'Base',
                   'drone4':'Base',
                   'drone5':'Base',
                   'drone6':'Base',
                   'drone7':'Base',
                   'drone8':'Base' } #intialize drones deployment status

        interval=10 #time in between energy checking [s]

        ###############################################################################################Drones parameters
        gravity=9.8 #Gravitational acceleration [m/s^2]

        m1 = 1  # Drone body mass [kg]
        m2 = 0.3  # Drone battery mass [kg]
        m3 = 0.1  # Drone camera mass [kg]
        m = m1 + m2 + m3  # [kg]

        lift_to_drag = 1.5  # Lift-to-drag ratio: typical range of aerodynamic efficiency for multi-rotor drones is 1-3
        efficiency = 0.5  # Rotors efficiency: typical range is 0.2-0.7

        p_avio = 15  # Power needed for avionics operations (on-board computations, communications, video-recording, gps, etc) [W]: 5-25 [W] of typical range
        v = 5  # Drone's speed relative to the wind [m/s]: 5-15 [m/s] is a typical speed range for video drones
        phi = 0.3  # windspeed-to-dronespeed ratio: typical range 0-0.6

        n = 4  # number of rotors
        s = 0.073  # rotor's area [m^2]: 0.049-0.113 [m^2] is a typical range for quad-rotor drones
        ro = 1.152062  # air density in Cosenza
        drag_coeff = 1.3  # vertical drag coefficient: 1-1.5 is a typical range for quad-rotor drones
        thr = ( ( (m*gravity) ** (3/2) / (2*n*ro*s) ** (1/2) ) + p_avio ) * interval * 20 #energy required to hover for an interval [J]

        recharge = { 'drone1':thr,
                     'drone2':thr,
                     'drone3':thr,
                     'drone4':thr,
                     'drone5':thr,
                     'drone6':thr,
                     'drone7':thr,
                     'drone8':thr } #initialize drones energy recharge thresholds (if residual energy is lower than the threshold, go back to supply station)

        while not (len(self.__access_points.keys())==4 and len(self.__drone_positions.keys())==8 and self.__num_endpoints==4): #wait network boot-up
            time.sleep(5) #check boot-up completion every 5 seconds

        while sum(datapath is not None for datapath in self.__datapaths.values())>0: #after network boot-up

            for dr,st in status.items(): #for every drone
                if self.__drone_positions[dr]==self.__base: #if the drone is currently at the base
                    if st=='Deployed' or st=='Returning': #if the drone was 'Returning' or 'Deployed' at the previous interval
                        status[dr]='Base' #update current drone state

                elif self.__drone_positions[dr]==self.__hover_positions[dr]: #if the drone is at the hovering position

                    if st=='Base' or st=='Moving': #if the drone has just reached hovering position
                        status[dr]='Hovering' #update current drone state

                        #First movement: vertical lift (along Z-axis). Assumptions: constant speed, thrust == weight + drag, zero lift
                        height=self.__hover_positions[dr][2] #[m] Along Z-Axis
                        e_lift=( ( (m*gravity) + ( n*ro*s*drag_coeff * (v**2) )/2 ) * (v/efficiency) + p_avio ) * (height/v) #[J] energy for first movement

                        #Second movement: horizontal flight along X-axis. Assumptions: constant speed, thrust == drag, weight == lift
                        distance1=self.__hover_positions[dr][0] #[m] Along X-axis
                        e_fly1=( ( (m*gravity) / (efficiency*lift_to_drag) ) + (p_avio/ (v/(1-phi)) ) ) * ( distance1/(1-phi) ) #[J] energy for second movement

                        #Third movement: horizontal flight along Y-axis. Assumptions: constant speed, thrust == drag, weight == lift
                        distance2=self.__hover_positions[dr][1] #[m] Along Y-axis
                        e_fly2=( ( (m*gravity) / (efficiency*lift_to_drag) ) + (p_avio/ (v/(1-phi)) ) ) * ( distance2/(1-phi) ) #[J] energy for third movement

                        self.__drone_energy[dr]-=(e_lift+e_fly1+e_fly2) #subtract from drone's residual energy the energy required to move to hovering position

                        #Fourth movement: like third
                        #Fifth movement: like second
                        #Last movement: vertical drop (along Z-axis). Assumptions: constant speed, thrust + drag == weight, zero lift
                        e_drop=( ( (m*gravity) + (n*ro*s*drag_coeff*(v**2) )/2 ) * (v/efficiency) + p_avio ) * (height/v) #[J] energy for first movement

                        recharge[dr]+=(e_fly2+e_fly1+e_drop) #update recharge threshold for drone, considering energy required to come back to base

                    elif status[dr]=='Hovering': #if the drone was already hovering at the previous interval
                        self.__drone_energy[dr]-=thr #subtract from drone's residual energy the energy required to hover for an interval

                        if self.__drone_energy[dr]<recharge[dr]: #if drones residual energy is under the threshold
                            with open(mob_log, "a") as log_file:
                                numb=int(dr[5]) #return the sixth character of the drone name, corresponding to its number

                                if numb<5: #for drones 1-4
                                    dr2=f'drone{numb+4}'
                                    log_file.write(f"   '---> Drone {dr2} tasked to move from: {self.__drone_positions[dr2]} to {self.__hover_positions[dr2]}\n")
                                    log_file.write(f"\n")

                                    self.send_command(json.dumps({"Drone": dr2, "Action": "Move to Hover",
                                                                  "Start Position": self.__drone_positions[dr2], #######
                                                                  "Final Position": self.__hover_positions[dr2]}))

                                else: #for drones 5-8
                                    dr2=f'drone{numb-4}'
                                    log_file.write(f"   '---> Drone {dr2} tasked to move from: {self.__drone_positions[dr2]} to {self.__hover_positions[dr2]}\n")
                                    log_file.write(f"\n")

                                    self.send_command(json.dumps({"Drone": dr2, "Action": "Move to Hover",
                                                                  "Start Position": self.__drone_positions[dr2], #######
                                                                  "Final Position": self.__hover_positions[dr2]}))

                                log_file.write(f"{dt.now()} -> Drone {dr}: residual energy {self.__drone_energy[dr]} [J] is below {recharge[dr]} [J]\n")
                                self.send_command(json.dumps({"Drone": dr, "Action": "Go Back to Base",
                                                              "Start Position": self.__drone_positions[dr],
                                                              "Final Position": self.__base}))

                            status[dr]='Returning' #update current drone state (it's returning to base)

                else: #if the drone is moving to hovering or coming back to base
                    if st=='Base':
                        status[dr]='Moving'
                    elif st=='Hovering':
                        status[dr]='Returning' #update current drone state

            time.sleep(interval) #wait for a hovering interval

        with open(log_path, "a") as log_file:
            log_file.write(f'{dt.now()} -> Network has shut down, thread terminates\n')
        return

    #############################################################################################Event-handler Functions
    '''Handles OF FeaturesReply message from connecting OVSAP (from HANDSHAKE_DISPATCHER to CONFIG_DISPATCHER) and install table-miss flow rule on AP
       @param FleetController object
       @param EventOFPSwitchFeatures event'''
    @set_ev_cls(ofp_event.EventOFPSwitchFeatures, CONFIG_DISPATCHER)
    def ap_features_handler(self, ev):
        datapath=ev.msg.datapath #get datapath instance (OVSAP) associated with event
        dpid=FleetController.dpid_to_hex(datapath.id) #DPID of current OVSAP (converted into hexadecimal string)

        try:
            ap=self.__dpids[datapath.id] #name of current OVSAP
            with open(log_path, "a") as log_file:
                log_file.write(f"{dt.now()} -> Install table-miss on connected AP {ap} (DPID={dpid}, {datapath.id})\n")

            ofproto=datapath.ofproto #adopted OpenFlow protocol version
            parser=datapath.ofproto_parser #to manage OF messages
            match=parser.OFPMatch() #empty match, matches all flows
            actions=[parser.OFPActionOutput(ofproto.OFPP_CONTROLLER,ofproto.OFPCML_NO_BUFFER)] #action: forward entire packet to controller's port
            self.add_flow(datapath, match,actions) #send FlowMod message to current OVSAP to install table-miss entry

            #Rule to match IPv4 packets with DSCP value 000011 (position notifications)
            dscp_value=0b000011 #DSCP value to match
            flag_match=parser.OFPMatch(eth_type=0x0800, ipv4_dst=self.__broadcastAddress, ip_dscp=dscp_value) #match IPv4 packets with specific DSCP value and dst
            flag_actions=[parser.OFPActionOutput(ofproto.OFPP_CONTROLLER, ofproto.OFPCML_NO_BUFFER)] #action: forward entire packet to controller's port
            self.add_flow(datapath, flag_match, flag_actions, priority=100) #send FlowMod message to OVSAP to install rule

            with open(log_path, "a") as log_file:
                log_file.write(f"{dt.now()} -> Install default rule for notification packest on connected AP {ap} (DPID={dpid}, {datapath.id})\n")
                log_file.write(f"\n")

        except KeyError as e:
            print(f"{e}: Unrecognized DPID {dpid}, {datapath.id}")

    '''Add/remove OVSAPs from dictionaries when they enter/exit the network
       @param FleetController object
       @param EventDP dp'''
    @set_ev_cls(EventDP, MAIN_DISPATCHER)
    def handle_ap_enter(self, dp):
        datapath=dp.dp #get datapath instance (OVSAP) associated with event
        dpid=FleetController.dpid_to_hex(datapath.id) #DPID of current OVSAP (converted into hexadecimal string)
        try:
            ap=self.__dpids[datapath.id] #name of current OVSAP
        except KeyError as e:
            print(f"{e}: Unrecognized DPID {dpid},{datapath.id}")

        enter=dp.enter #flag: True if current OVSAP is entering the network, False if OVSAP is disconnecting
        ports_list=dp.ports #retrieves a list of L2 ports available on current AP

        if enter: #if current AP is joining the network (flag True)
            with open(log_path, "a") as log_file:
                log_file.write(f"{dt.now()} -> AP {ap} (DPID={dpid}, {datapath.id}) entered the network\n")
                for port in ports_list:
                    log_file.write(f"   '---> Port name: {port.name}, port_no: {port.port_no}, hw_addr: {port.hw_addr}\n")
                log_file.write(f"\n")

            self.__datapaths[datapath.id]=datapath #add datapath reference to dictionary when connecting

            self.__access_points[ap]={} #initialize dictionary of AP ports
            for port in ports_list:
                self.__access_points[ap][port.name]=port.port_no #add OVSAP port list to dictionary when connecting

        else: #if the AP is exiting the network (flag False)
            with open(log_path, "a") as log_file:
                log_file.write(f'{dt.now()} -> AP {ap} (DPID={dpid}, {datapath.id}) exiting the network\n')

            self.__access_points[ap]=None #remove list of AP ports from dictionary
            self.__datapaths[datapath.id]=None #remove reference to datapath object from dictionary

    '''Add endpoints and stations (Host devices) to dictionaries when they join the network. If network boot-up is completed it triggers proactive 
       computation and installation of video streaming traffic rules
       @param FleetController object
       @param EventHostAdd ev'''
    @set_ev_cls(EventHostAdd)
    def event_host_add_handler(self, ev):
        host=ev.host #host associated with event: <Host<str mac, port=Port<int dpid, int port_no, LIVE>

        try:
            if host.mac in self.__drone_macs.keys():
                host_name=self.__drone_macs[host.mac] #name of current host
                port_name=f"{host_name}-wlan0"

                with open(log_path, "a") as log_file:
                    log_file.write(f"{dt.now()} -> Drone {host_name} entered the network\n")
                    log_file.write(f"   '---> Port name: {port_name}, port_no: {0}, hw_addr: {host.mac}\n")
                    log_file.write(f"\n")

                capacity = 5000  # Battery capacity [mAh]
                voltage = 15.4  # Battery voltage [V]

                e_battery=capacity*voltage*3.6 #Full-battery energy [J]

                self.__drone_positions[host_name]=self.__base #initialize drone's position
                self.__drone_energy[host_name]=e_battery #initialize drone's residual energy

            elif host.mac in self.__endpoint_macs.keys():
                host_name=self.__endpoint_macs[host.mac] #name of current host
                port_name=f"{host_name}-eth1"

                with open(log_path, "a") as log_file:
                    log_file.write(f"{dt.now()} -> Endpoint {host_name} entered the network\n")
                    log_file.write(f"   '---> Port name: {port_name}, port_no: {1}, hw_addr: {host.mac}\n")
                    log_file.write(f"\n")

                self.__num_endpoints+=1 #count endpoints

            else:
                raise KeyError

        except KeyError as e:
            print(f"{e}: Unrecognized MAC {host.mac}")

        if len(self.__access_points.keys())==4 and len(self.__drone_positions.keys())==8 and self.__num_endpoints==4: #if all expected nodes are connected
            with open(log_path, "a") as log_file:
                log_file.write(f"{dt.now()} -> Network boot-up completed \n")

                for ap in self.__access_points.keys():
                    log_file.write(f"{dt.now()} -> AP {ap}, ports: {self.__access_points[ap]}\n")
                    if ap=='ap1':
                        log_file.write(f"   '---> DPID: {FleetController.dpid_to_hex(13)}, Datapath: {self.__datapaths[13]}\n")
                    elif ap=='ap2':
                        log_file.write(f"   '---> DPID: {FleetController.dpid_to_hex(14)}, Datapath: {self.__datapaths[14]}\n")
                    elif ap=='ap3':
                        log_file.write(f"   '---> DPID: {FleetController.dpid_to_hex(15)}, Datapath: {self.__datapaths[15]}\n")
                    else: #ap4
                        log_file.write(f"   '---> DPID: {FleetController.dpid_to_hex(16)}, Datapath: {self.__datapaths[16]}\n")
                log_file.write(f"\n")

                for drone in self.__drone_positions.keys():
                    log_file.write(f"{dt.now()} -> Drone {drone}, Position: {self.__drone_positions[drone]}\n")
                    log_file.write(f"   '---> Residual Energy: {self.__drone_energy[drone]} [J]\n")
                log_file.write(f"\n")

                for mac in self.__endpoint_macs.keys():
                    log_file.write(f"{dt.now()} -> Endpoint {self.__endpoint_macs[mac]}\n")
                log_file.write(f"\n")

            with open(mob_log, "w") as log_file:
                log_file.write(f"{dt.now()} -> Initial Positions\n")

                for d, p in self.__drone_positions.items():
                    if p==self.__hover_positions[d]:
                        status='Deployed'
                    elif p==self.__base:
                        status='Base'
                    else:
                        status='Moving'
                    log_file.write(f"   '---> {d}:{p}. Status: {status}. Remaining energy: {self.__drone_energy[d]} [J]\n")
                log_file.write(f"\n")

            self.proactive_streaming() #computes and installs proactively video streaming rules (group rules, meter rules, and flow rules)
            self.proactive_broadcast() #computes and installs proactively IPv4 broadcast rules (group rules and flow rules)

    '''Called when an OVSAP sends an OF Packet-In to the controller
       @param FleetController object
       @param EventOFPPacketIn ev'''
    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def _packet_in_handler(self, ev):
        if ev.msg.msg_len<ev.msg.total_len:
            self.logger.debug("packet truncated: only %s of %s bytes", ev.msg.msg_len, ev.msg.total_len)

        msg=ev.msg #get Packet-In message associated with event
        datapath=msg.datapath #OVSAP associated with event (datapath reference)
        inport=msg.match['in_port']

        pkt=packet.Packet(msg.data) #message encapsulated in packet-In
        assert datapath in self.__datapaths.values(), f"PacketIn received from unknown AP {datapath.id}"

        eth=pkt.get_protocols(ethernet.ethernet)[0] #get Ethernet-like frame from encapsulated packet
        src_mac=eth.src #get source MAC address from frame

        if eth.ethertype==ether_types.ETH_TYPE_LLDP: #if L3 protocol is LLDP
            return #ignore LLDP packets
        if eth.ethertype!=ether_types.ETH_TYPE_ARP and eth.ethertype!=ether_types.ETH_TYPE_IP:
            self.logger.debug(f"Unauthorized type of traffic {eth.ethertype}")
            return #we only provide flow rules for ARP traffic and IPv4 traffic

        elif eth.ethertype==ether_types.ETH_TYPE_ARP: #if L3 protocol is ARP
            pkt3=pkt.get_protocol(arp.arp) #get ARP packet (None if not ARP)
            src_ip=pkt3.src_ip #get source IPv4 address from packet
            dst_ip=pkt3.dst_ip #get destination IPv4 address from packet
            self.reactive_arp(msg, datapath, inport, src_mac, src_ip, dst_ip)

        else: #eth.ethertype!=ether_types.ETH_TYPE_IP (L3 protocol is IPv4)
            pkt3=pkt.get_protocol(ipv4.ipv4) #get IPv4 packet (None if not IPv4)
            src_ip=pkt3.src #get source IPv4 address from packet
            dst_ip=pkt3.dst #get destination IPv4 address from packet

            dscp_value=pkt3.tos>>2 #Extract DSCP value from the TOS field of the encapsulated IPv4 packet (upper 6 bits)
            self.logger.debug(f"IPv4 packet DSCP value: {dscp_value:06b} (decimal: {dscp_value})")
            if dscp_value==0b000011: #Check if the DSCP value matches 000011
                pkt4=pkt.get_protocol(udp.udp) #extract L4 datagram from message

                payload=msg.data #Raw packet bytes
                udp_payload=payload[-(pkt4.total_length-8):] #UDP payload length = total - header (8 bytes)

                decoded_paylaod=udp_payload.decode(errors="ignore")

                position=decoded_paylaod.split('/')[0].split(':')[1]
                occupation=decoded_paylaod.split('/')[1].split(':')[1]

                self.update_pos(src_mac, position) #update's drone's position
                self.update_rate(datapath.id, occupation)

            else:
                if dst_ip==self.__broadcastAddress:
                    return
                self.reactive_ipv4(msg, datapath, inport, src_ip, dst_ip)

    ###################################################################################################Routing Functions
    '''Defines flow rules and group rules to route video streaming from drones to endpoints
       @param FleetController object'''
    def proactive_streaming(self):
        ap_configs={} #######################################################################access points configuration
        for ap_name, port_list in self.__access_points.items(): #dynamic access point configuration
            wireless=[] #AP WiFi interfaces
            wired=[] #AP Ethernet interfaces

            for port_name_bytes, port_no in port_list.items():
                port_name=port_name_bytes.decode() if isinstance(port_name_bytes, bytes) else port_name_bytes
                if 'wlan' in port_name:
                    wireless.append(port_name)
                elif 'eth' in port_name:
                    if not 'eth6' in port_name: #exclude extra ethernet interface
                        wired.append(port_name)

            for k,v in self.__dpids.items():
                if v==ap_name:
                    datapath_id=k

            ap_configs[ap_name] = { 'dpid': datapath_id,
                                    'wireless': wireless,
                                    'wired': wired } #current AP configuration

        for ap_name, ap_data in ap_configs.items(): ##########################computing and installing rules for each AP
            dpid=ap_data['dpid']
            ap=self.__datapaths.get(dpid)

            with open(log_path, "a") as log_file:
                log_file.write(f"{dt.now()} -> Installing streaming rules for {ap_name}, {ap}\n")
                log_file.write(f"   '---> Configurations: {ap_data}\n")
                log_file.write(f"\n")

            if not ap:
                print(f"[ERROR] Datapath not found for {ap_name} (DPID {FleetController.dpid_to_hex(dpid)}, {dpid})")
                continue

            ofproto=ap.ofproto
            parser=ap.ofproto_parser

            #####################################################################Rules from wlan1 to eth2/eth3/eth4/eth5
            for wlan_iface in ap_data['wireless']: #for each wireless interface
                try:
                    in_port=self.__access_points[ap_name][wlan_iface.encode()] #ingress port number: WiFi Interface
                    with open(log_path, "a") as log_file:
                        log_file.write(f"   '---> WiFi Interface: {wlan_iface} -> in_port_ {in_port}\n")

                except KeyError:
                    print(f"[ERROR] WiFi interface '{wlan_iface}' missing on {ap_name}")
                    continue

                ######################################################################################Create Group Rules
                buckets_L=[] #create bucket of action lists (low-quality)
                buckets_H=[] #create bucket of action lists (high-quality)
                for eth_iface in ap_data['wired']: #for each Ethernet interface
                    try:
                        out_port=self.__access_points[ap_name][eth_iface.encode()] #output port number: Ethernet Interface
                        action=parser.OFPActionOutput(out_port)

                        if eth_iface[7]=='2': #eth2
                            if ap_name=='ap1' or ap_name=='ap3':
                                buckets_H.append(parser.OFPBucket(actions=[action]))
                            else: #ap2 and ap4
                                buckets_L.append(parser.OFPBucket(actions=[action]))

                        elif eth_iface[7]=='3': #eth3
                            if ap_name=='ap1':
                                buckets_L.append(parser.OFPBucket(actions=[action]))
                            else: #ap2, ap3, and ap4
                                buckets_H.append(parser.OFPBucket(actions=[action]))

                        elif eth_iface[7]=='4': #eth4
                            if ap_name=='ap3' or ap_name=='ap4':
                                buckets_L.append(parser.OFPBucket(actions=[action]))
                            else: #ap1 and ap2
                                buckets_H.append(parser.OFPBucket(actions=[action]))

                        else: #eth5
                            if ap_name=='ap4':
                                buckets_H.append(parser.OFPBucket(actions=[action]))
                            else: #ap1, ap2, and ap3
                                buckets_L.append(parser.OFPBucket(actions=[action]))

                        with open(log_path, "a") as log_file:
                            log_file.write(f"   '---> Action: {eth_iface} -> out_port {out_port}\n")

                    except KeyError:
                        print(f"[ERROR] Ethernet Interface '{eth_iface}' missing on {ap_name}")

                group_id_H=1 #unique group ID
                group_id_L=2 #unique group id

                reqH=parser.OFPGroupMod( datapath=ap,
                                        command=ofproto.OFPGC_ADD,
                                        type_=ofproto.OFPGT_ALL,
                                        group_id=group_id_H,
                                        buckets=buckets_H) #group mod message
                ap.send_msg(reqH) #controller issues group mode message to current AP
                with open(log_path, "a") as log_file:
                    log_file.write(f"   '---> Group rule {group_id_H} created on {ap_name}, {ap}\n")
                    log_file.write(f"   '---> Buckets: {buckets_H}\n")
                    log_file.write(f"\n")

                reqL=parser.OFPGroupMod( datapath=ap,
                                         command=ofproto.OFPGC_ADD,
                                         type_=ofproto.OFPGT_ALL,
                                         group_id=group_id_L,
                                         buckets=buckets_L) #group mod message
                ap.send_msg(reqL) #controller issues group mode message to current AP
                with open(log_path, "a") as log_file:
                    log_file.write(f"   '---> Group rule {group_id_L} created on {ap_name}, {ap}\n")
                    log_file.write(f"   '---> Buckets: {buckets_L}\n")
                    log_file.write(f"\n")

                ######################################################################################Create Meter Rules
                supported_rateH=1 #[bps]
                for val in self.__quality['high']:
                    supported_rateH*=val
                supported_rateH=int(supported_rateH/1000) #[kbps]

                bandsH=[parser.OFPMeterBandDrop(rate=supported_rateH, type_=1, len_=16)]
                meter_modH=parser.OFPMeterMod(datapath=ap, command=ofproto.OFPMC_ADD,
                                              flags=ofproto.OFPMF_KBPS,
                                              meter_id=1, #unique meter ID
                                              bands=bandsH) #meter_mod message

                ap.send_msg(meter_modH)

                with open(log_path, "a") as log_file:
                    log_file.write(f"   '---> Meter rule {1} created on {ap_name}, {ap}\n")
                    log_file.write(f"   '---> Bands: {bandsH}\n")

                supported_rateL=1 #[bps]
                for val in self.__quality['low']:
                    supported_rateL*=val
                supported_rateL=int(supported_rateL/1000) #[kbps]

                bandsL=[parser.OFPMeterBandDrop(rate=supported_rateL, type_=1, len_=16)]
                meter_modL=parser.OFPMeterMod(datapath=ap, command=ofproto.OFPMC_ADD,
                                              flags=ofproto.OFPMF_KBPS,
                                              meter_id=2, #unique meter ID
                                              bands=bandsL) #meter_mod message

                ap.send_msg(meter_modL)

                with open(log_path, "a") as log_file:
                    log_file.write(f"   '---> Meter rule {2} created on {ap_name}, {ap}\n")
                    log_file.write(f"   '---> Bands: {bandsL}\n")

                #######################################################################################Create Flow Rules
                for ip in self.__ip_groups[ap_name][:2]: #for each drone IPv4 address connected to current AP
                    match=parser.OFPMatch(eth_type=0x0800, in_port=in_port, ipv4_src=ip, ipv4_dst=self.__broadcastAddress,
                                          ip_dscp=0b000000, ip_proto=17, udp_dst=8888) #match for high quality streams
                    action_udp=parser.OFPActionSetField(udp_dst=int(f"123{ip[10]}"))
                    actions=[action_udp, parser.OFPActionGroup(group_id_H)] #actions: change destination L4 Port and apply group
                    self.add_flow(datapath=ap, match=match, actions=actions, priority=16, meter_id=1) #install streaming flow rule on current AP (high quality)

                    with open(log_path, "a") as log_file:
                        log_file.write(f"   '---> Flow rule {match} installed on {ap_name},{ap}\n")
                        log_file.write(f"   '---> Actions: {actions} \n")
                        log_file.write(f"\n")

                    match=parser.OFPMatch(eth_type=0x0800, in_port=in_port, ipv4_src=ip, ipv4_dst=self.__broadcastAddress,
                                          ip_dscp=0b000000, ip_proto=17, udp_dst=7777) #match for low quality streams
                    action_udp=parser.OFPActionSetField(udp_dst=int(f"123{ip[10]}"))
                    actions=[action_udp, parser.OFPActionGroup(group_id_L)] #actions: change destination L4 Port and apply group
                    self.add_flow(datapath=ap, match=match, actions=actions, priority=16, meter_id=2) #install streaming flow rule on current AP (low quality)

                    with open(log_path, "a") as log_file:
                        log_file.write(f"   '---> Flow rule {match} installed on {ap_name},{ap}\n")
                        log_file.write(f"   '---> Actions: {actions} \n")
                        log_file.write(f"\n")

            ######################################################################Flow Rules from eth3/eht4/eth5 to eth2
            for i in range(3,6):
                in_eth=f'{ap_name}-eth{i}' #input port name
                out_eth=f'{ap_name}-eth2' #output port name (towards endpoint)

                try:
                    in_port=self.__access_points[ap_name][in_eth.encode()] #ingress port number: Ethernet Interface towards other AP
                    with open(log_path, "a") as log_file:
                        log_file.write(f"   '---> Ethernet Interface: {in_eth} -> in_port_ {in_port}\n")

                except KeyError:
                    print(f"[ERROR] Ethernet interface '{in_eth}' missing on {ap_name}")
                    continue

                try:
                    out_port=self.__access_points[ap_name][out_eth.encode()] #output port number: Ethernet Interface towards endpoint
                    with open(log_path, "a") as log_file:
                        log_file.write(f"   '---> Action: {out_eth} -> out_port {out_port}\n")

                except KeyError:
                    print(f"[ERROR] Ethernet Interface '{out_eth}' missing on {ap_name}")

                match=parser.OFPMatch(eth_type=0x0800, in_port=in_port, ipv4_dst=self.__broadcastAddress, ip_dscp=0b000000, ip_proto=17)
                actions=[parser.OFPActionOutput(out_port)]
                self.add_flow(datapath=ap, match=match, actions=actions, priority=16) #install streaming flow rule on current AP

                with open(log_path, "a") as log_file:
                    log_file.write(f"   '---> Flow rule {match} installed on {ap_name},{ap}\n")
                    log_file.write(f"   '---> Actions: {actions} \n")
                    log_file.write(f"\n")

    '''Defines flow rules and group rules to route generic IPv4 broadcast traffic
       @param FleetController object'''
    def proactive_broadcast(self):
        ap_configs={} #######################################################################access points configuration
        for ap_name, port_list in self.__access_points.items(): #dynamic access point configuration
            wireless=[] #AP WiFi interfaces
            wired=[] #AP Ethernet interfaces

            for port_name_bytes, port_no in port_list.items():
                port_name=port_name_bytes.decode() if isinstance(port_name_bytes, bytes) else port_name_bytes
                if 'wlan' in port_name:
                    wireless.append(port_name)
                elif 'eth' in port_name:
                    if not 'eth6' in port_name: #exclude extra ethernet interface
                        wired.append(port_name)

            for k, v in self.__dpids.items():
                if v==ap_name:
                    datapath_id=k

            ap_configs[ap_name]={'dpid': datapath_id,
                                 'wireless': wireless,
                                 'wired': wired} #current AP configuration

        for ap_name, ap_data in ap_configs.items(): ##########################computing and installing rules for each AP
            dpid=ap_data['dpid']
            ap=self.__datapaths.get(dpid)

            with open(log_path, "a") as log_file:
                log_file.write(f"{dt.now()} -> Installing broadcast rules for {ap_name}, {ap}\n")
                log_file.write(f"   '---> Configurations: {ap_data}\n")
                log_file.write(f"\n")

            if not ap:
                print(f"[ERROR] Datapath not found for {ap_name} (DPID {FleetController.dpid_to_hex(dpid)}, {dpid})")
                continue

            ofproto=ap.ofproto
            parser=ap.ofproto_parser

            #####################################################################Rules from wlan1 to eth2/eth3/eth4/eth5
            for wlan_iface in ap_data['wireless']: #for each wireless interface
                try:
                    in_port=self.__access_points[ap_name][wlan_iface.encode()] #ingress port number: WiFi Interface
                    with open(log_path, "a") as log_file:
                        log_file.write(f"   '---> WiFi Interface: {wlan_iface} -> in_port_ {in_port}\n")
                except KeyError:
                    print(f"[ERROR] WiFi interface '{wlan_iface}' missing on {ap_name}")
                    continue

                #######################################################################################Create Group Rule
                buckets=[] #create bucket of action lists
                for eth_iface in ap_data['wired']: #for each Ethernet interface
                    try:
                        out_port=self.__access_points[ap_name][eth_iface.encode()] #output port number: Ethernet Interface
                        action=parser.OFPActionOutput(out_port)
                        buckets.append(parser.OFPBucket(actions=[action]))
                        with open(log_path, "a") as log_file:
                            log_file.write(f"   '---> Action: {eth_iface} -> out_port {out_port}\n")

                    except KeyError:
                        print(f"[ERROR] Ethernet Interface '{eth_iface}' missing on {ap_name}")

                group_id=11 #unique group ID
                req=parser.OFPGroupMod(datapath=ap,
                                       command=ofproto.OFPGC_ADD,
                                       type_=ofproto.OFPGT_ALL,
                                       group_id=group_id,
                                       buckets=buckets) #group mod message
                ap.send_msg(req) #controller issues group mode message to current AP
                with open(log_path, "a") as log_file:
                    log_file.write(f"   '---> Group rule {group_id}:{buckets} created on {ap_name}\n")

                ########################################################################################Create Flow Rule
                match=parser.OFPMatch(eth_type=0x0800, in_port=in_port, ipv4_dst=self.__broadcastAddress, ip_dscp=0b000000)
                actions=[parser.OFPActionGroup(group_id)] #actions: apply group
                self.add_flow(datapath=ap, match=match, actions=actions, priority=12) #install broadcast flow rule on current AP

                with open(log_path, "a") as log_file:
                    log_file.write(f"   '---> Flow rule {match} installed on {ap_name},{ap}\n")
                    log_file.write(f"   '---> Actions {actions} \n")
                    log_file.write(f"\n")

            #####################################################################Rules from eth2 to wlan1/eth3/eth4/eth5
            in_eth=f'{ap_name}-eth2' #input port name
            try:
                in_port=self.__access_points[ap_name][in_eth.encode()] #ingress port number: Ethernet Interface towards other AP
                with open(log_path, "a") as log_file:
                    log_file.write(f"   '---> Ethernet Interface: {in_eth} -> in_port_ {in_port}\n")
            except KeyError:
                print(f"[ERROR] Ethernet interface '{in_eth}' missing on {ap_name}")
                continue

            ###########################################################################################Create Group Rule
            buckets=[] #create bucket of action lists
            for wlan_iface in ap_data['wireless']: #for each wireless interface
                try:
                    out_port=self.__access_points[ap_name][wlan_iface.encode()] #output port number: WiFi Interface
                    action=parser.OFPActionOutput(out_port)
                    buckets.append(parser.OFPBucket(actions=[action]))
                    with open(log_path, "a") as log_file:
                        log_file.write(f"   '---> Action: {wlan_iface} -> out_port {out_port}\n")

                except KeyError:
                    print(f"[ERROR] Ethernet Interface '{wlan_iface}' missing on {ap_name}")

            for eth_iface in ap_data['wired']: #for each Ethernet interface
                try:
                    if eth_iface==in_eth: #ignore input interface
                        continue
                    out_port=self.__access_points[ap_name][eth_iface.encode()] #output port number: Ethernet Interface
                    action=parser.OFPActionOutput(out_port)
                    buckets.append(parser.OFPBucket(actions=[action]))
                    with open(log_path, "a") as log_file:
                        log_file.write(f"   '---> Action: {eth_iface} -> out_port {out_port}\n")

                except KeyError:
                    print(f"[ERROR] Ethernet Interface '{eth_iface}' missing on {ap_name}")

            group_id=12 #unique group ID
            req=parser.OFPGroupMod(datapath=ap,
                                   command=ofproto.OFPGC_ADD,
                                   type_=ofproto.OFPGT_ALL,
                                   group_id=group_id,
                                   buckets=buckets) #group mod message
            ap.send_msg(req) #controller issues group mode message to current AP
            with open(log_path, "a") as log_file:
                log_file.write(f"   '---> Group rule {group_id}:{buckets} created on {ap_name}\n")

            ############################################################################################Create Flow Rule
            match=parser.OFPMatch(eth_type=0x0800, in_port=in_port, ipv4_dst=self.__broadcastAddress, ip_dscp=0b000000)
            actions=[parser.OFPActionGroup(group_id)] #actions: apply group
            self.add_flow(datapath=ap, match=match, actions=actions, priority=12) #install streaming flow rule on current AP

            with open(log_path, "a") as log_file:
                log_file.write(f"   '---> Flow rule {match} installed on {ap_name},{ap}\n")
                log_file.write(f"   '---> Actions {actions} \n")
                log_file.write(f"\n")

            ###########################################################################From eth3/eth4/eth5 to wlan1/eth2
            wlan_iface=f'{ap_name}-wlan1' #WiFi Interface
            eth_iface=f'{ap_name}-eth2' #Ethernet Interface towards endpoint

            ###########################################################################################Create Group Rule
            buckets=[] #create bucket of action lists
            try:
                out_port_w=self.__access_points[ap_name][wlan_iface.encode()] #output port number: WiFi Interface
                action_w=parser.OFPActionOutput(out_port_w)

                out_port_e=self.__access_points[ap_name][eth_iface.encode()] #output port number: Ethernet Interface towards endpoint
                action_e=parser.OFPActionOutput(out_port_e)

                buckets.append(parser.OFPBucket(actions=[action_w, action_e]))
                with open(log_path, "a") as log_file:
                    log_file.write(f"   '---> Action: {wlan_iface} -> out_port {out_port_w}\n")
                    log_file.write(f"   '---> Action: {eth_iface} -> out_port {out_port_e}\n")

            except KeyError:
                print(f"[ERROR] nterface missing on {ap_name}")

            group_id=13 #unique group ID
            req=parser.OFPGroupMod(datapath=ap,
                                   command=ofproto.OFPGC_ADD,
                                   type_=ofproto.OFPGT_ALL,
                                   group_id=group_id,
                                   buckets=buckets) #group mod message
            ap.send_msg(req) #controller issues group mode message to current AP
            with open(log_path, "a") as log_file:
                log_file.write(f"   '---> Group rule {group_id}:{buckets} created on {ap_name}\n")

            for i in range(3,6): ######################################################################Create Flow Rules
                in_eth=f'{ap_name}-eth{i}' #input port name

                try:
                    in_port=self.__access_points[ap_name][in_eth.encode()] #ingress port number: Ethernet Interface towards other AP
                    with open(log_path, "a") as log_file:
                        log_file.write(f"   '---> Ethernet Interface: {in_eth} -> in_port_ {in_port}\n")

                except KeyError:
                    print(f"[ERROR] Ethernet interface '{in_eth}' missing on {ap_name}")
                    continue


                match=parser.OFPMatch(eth_type=0x0800, in_port=in_port, ipv4_dst=self.__broadcastAddress, ip_dscp=0b000000)
                actions=[parser.OFPActionGroup(group_id)] #actions: apply group
                self.add_flow(datapath=ap, match=match, actions=actions, priority=12) #install streaming flow rule on current AP

                with open(log_path, "a") as log_file:
                    log_file.write(f"   '---> Flow rule {match} installed on {ap_name},{ap}\n")
                    log_file.write(f"   '---> Actions: {actions} \n")
                    log_file.write(f"\n")

    '''Defines flow rules for unicast ARP messages between host devices (drones and endpoints)
       @param FleetController object
       @param MsgBase message (Packet-In message)
       @param Datapath ap (reference to OVSAP)
       @param int inport
       @param str src_mac
       @param str src
       @param str dst'''
    def reactive_arp(self, message, ap, inport, src_mac, src, dst):
        dpid=ap.id
        ap_name=self.__dpids[dpid]

        with open(log_path, "a") as log_file:
            log_file.write(f"{dt.now()} -> Installing ARP rules on {ap_name}: {ap}, ({FleetController.dpid_to_hex(dpid), dpid}\n")
            log_file.write(f"   '---> Routing ARP {src_mac} : {src} -> {dst}\n")
            log_file.write(f"   '---> Input port number: {inport}\n")

        if dst in self.__ip_groups['ap1']: #the destination node is connected to AP1
            if ap_name=='ap1':
                if dst!=self.__ip_groups['ap1'][2]: #if destination is a drone
                    out_port_name=f'{ap_name}-wlan1' #egress port WiFi
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
                else: #destination is endpoint1
                    out_port_name=f'{ap_name}-eth2' #egress Ethernet port (towards endpoint1)
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            else: #ap2, ap3, or ap4
                out_port_name=f'{ap_name}-eth3' #egress port Ethernet (towards AP1 in all other APs)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number

        elif dst in self.__ip_groups['ap2']: #the destination node is connected to AP2
            if ap_name=='ap2':
                if dst!=self.__ip_groups['ap2'][2]: #if destination is a drone
                    out_port_name=f'{ap_name}-wlan1' #egress port WiFi
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
                else: #destination is endpoint2
                    out_port_name=f'{ap_name}-eth2' #egress Ethernet port (towards endpoint2)
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            elif ap_name=='ap1':
                out_port_name=f'{ap_name}-eth3' #egress port Ethernet (towards AP2)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            elif ap_name=='ap3':
                out_port_name=f'{ap_name}-eth4' #egress port Ethernet (towards AP2)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            else: #ap_name=='ap4'
                out_port_name=f'{ap_name}-eth4' #egress port Ethernet (towards AP2)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number

        elif dst in self.__ip_groups['ap3']: #the destination node is connected to AP3
            if ap_name=='ap3':
                if dst!=self.__ip_groups['ap3'][2]: #if destination is a drone
                    out_port_name=f'{ap_name}-wlan1' #egress port WiFi
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
                else: #destination is endpoint3
                    out_port_name=f'{ap_name}-eth2' #egress Ethernet port (towards endpoint3)
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            elif ap_name=='ap1' or ap_name=='ap2':
                out_port_name=f'{ap_name}-eth4' #egress port Ethernet (towards AP3)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            else: #ap_name=='ap4'
                out_port_name=f'{ap_name}-eth5' #egress port Ethernet (towards AP3)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number

        elif dst in self.__ip_groups['ap4']: #the destination node is connected to AP4
            if ap_name=='ap4':
                if dst!=self.__ip_groups['ap4'][2]: #if destination is a drone
                    out_port_name=f'{ap_name}-wlan1' #egress port WiFi
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
                else: #destination is endpoint4
                    out_port_name=f'{ap_name}-eth2' #egress Ethernet port (towards endpoint4)
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            else: #ap1, ap2, or ap3
                out_port_name=f'{ap_name}-eth5' #egress port Ethernet (towards AP4)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number

        elif dst in self.__spurious_ip:
            if dst=='192.168.1.21':
                out_port_name=f'ap1-wlan1' #egress port WiFi
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            elif dst=='192.168.1.22':
                out_port_name=f'ap1-eth6' #egress port ethernet
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number

        else:
            print(f"Invalid destination IPv4 address {dst}\n")
            return

        with open(log_path, "a") as log_file:
            log_file.write(f"   '---> Output port {out_port_name} ({out_port})\n")

        ofproto=ap.ofproto
        parser=ap.ofproto_parser

        match=parser.OFPMatch(eth_type=0x0806, in_port=inport, eth_src=src_mac, arp_spa=src, arp_tpa=dst) #match on ARP protocol, L2 input port and src/dst IPv4
        actions=[parser.OFPActionOutput(out_port)] #actions: forward out designated exit port
        self.add_flow(datapath=ap, match=match, actions=actions, priority=8) #install flow rule on current AP

        with open(log_path, "a") as log_file:
            log_file.write(f"   '---> Flow rule {match} installed on {ap_name},{ap}\n")
            log_file.write(f"\n")

        #handle message inside Packet-In with flow mod
        data=None #content of packet out message
        if message.buffer_id==ofproto.OFP_NO_BUFFER: #if no buffer ID was specified in the Packet-In message
            data=message.data #append to packet out the notification packet that has triggered packet in
        out=parser.OFPPacketOut(datapath=ap, buffer_id=message.buffer_id, in_port=inport, actions=actions, data=data) #packet out message
        ap.send_msg(out) #controller send packet out to OVSAP

    '''Defines flow rules for unicast IPv4 messages between host devices (drones and endpoints)
       @param FleetController object
       @param MsgBase message (Packet-In message)
       @param Datapath ap (reference to OVSAP)
       @param int inport
       @param str src
       @param str dst'''
    def reactive_ipv4(self, message, ap, inport, src, dst):
        dpid=ap.id
        ap_name=self.__dpids[dpid]

        with open(log_path, "a") as log_file:
            log_file.write(f"{dt.now()} -> Installing IPv4 rules on {ap_name}: {ap}, ({FleetController.dpid_to_hex(dpid), dpid}\n")
            log_file.write(f"   '---> Routing IPv4 {src} -> {dst}\n")
            log_file.write(f"   '---> Input port number: {inport}\n")

        if dst in self.__ip_groups['ap1']: #the destination node is connected to AP1
            if ap_name=='ap1':
                if dst!=self.__ip_groups['ap1'][2]: #if destination is a drone
                    out_port_name=f'{ap_name}-wlan1' #egress port WiFi
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
                else: #destination is endpoint1
                    out_port_name=f'{ap_name}-eth2' #egress Ethernet port (towards endpoint1)
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            else: #ap2, ap3, or ap4
                out_port_name=f'{ap_name}-eth3' #egress port Ethernet (towards AP1 in all other APs)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number

        elif dst in self.__ip_groups['ap2']: #the destination node is connected to AP2
            if ap_name=='ap2':
                if dst!=self.__ip_groups['ap2'][2]: #if destination is a drone
                    out_port_name=f'{ap_name}-wlan1' #egress port WiFi
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
                else: #destination is endpoint2
                    out_port_name=f'{ap_name}-eth2' #egress Ethernet port (towards endpoint2)
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            elif ap_name=='ap1':
                out_port_name=f'{ap_name}-eth3' #egress port Ethernet (towards AP2)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            elif ap_name=='ap3':
                out_port_name=f'{ap_name}-eth4' #egress port Ethernet (towards AP2)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            else: #ap_name=='ap4'
                out_port_name=f'{ap_name}-eth4' #egress port Ethernet (towards AP2)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number

        elif dst in self.__ip_groups['ap3']: #the destination node is connected to AP3
            if ap_name=='ap3':
                if dst!=self.__ip_groups['ap3'][2]: #if destination is a drone
                    out_port_name=f'{ap_name}-wlan1' #egress port WiFi
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
                else: #destination is endpoint3
                    out_port_name=f'{ap_name}-eth2' #egress Ethernet port (towards endpoint3)
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            elif ap_name=='ap1' or ap_name=='ap2':
                out_port_name=f'{ap_name}-eth4' #egress port Ethernet (towards AP3)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            else: #ap_name=='ap4'
                out_port_name=f'{ap_name}-eth5' #egress port Ethernet (towards AP3)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number

        elif dst in self.__ip_groups['ap4']: #the destination node is connected to AP4
            if ap_name=='ap4':
                if dst!=self.__ip_groups['ap4'][2]: #if destination is a drone
                    out_port_name=f'{ap_name}-wlan1' #egress port WiFi
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
                else: #destination is endpoint4
                    out_port_name=f'{ap_name}-eth2' #egress Ethernet port (towards endpoint4)
                    out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            else: #ap1, ap2, or ap3
                out_port_name=f'{ap_name}-eth5' #egress port Ethernet (towards AP4)
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number

        elif dst in self.__spurious_ip:
            if dst=='192.168.1.21':
                out_port_name=f'ap1-wlan1' #egress port WiFi
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number
            elif dst=='192.168.1.22':
                out_port_name=f'ap1-eth6' #egress port ethernet
                out_port=self.__access_points[ap_name][out_port_name.encode()] #get output port number

        else:
            print(f"Invalid destination IPv4 address {dst}\n")
            return

        with open(log_path, "a") as log_file:
            log_file.write(f"   '---> Output port {out_port_name} ({out_port})\n")

        ofproto=ap.ofproto
        parser=ap.ofproto_parser

        match=parser.OFPMatch(eth_type=0x0800, in_port=inport, ipv4_src=src, ipv4_dst=dst) #match on IPv4 protocol, L2 input/output port and src/dst IPv4
        actions=[parser.OFPActionOutput(out_port)] #actions: forward out designated exit port
        self.add_flow(datapath=ap, match=match, actions=actions, priority=10) #install flow rule on current AP

        with open(log_path, "a") as log_file:
            log_file.write(f"   '---> Flow rule {match} installed on {ap_name},{ap}\n")
            log_file.write(f"\n")

        #handle message inside Packet-In with flow mod
        data=None #content of packet out message
        if message.buffer_id==ofproto.OFP_NO_BUFFER: #if no buffer ID was specified in the Packet-In message
            data=message.data #append to packet out the notification packet that has triggered packet in
        out=parser.OFPPacketOut(datapath=ap, buffer_id=message.buffer_id, in_port=inport, actions=actions, data=data) #packet out message
        ap.send_msg(out) #controller send packet out to OVSAP

    #####################################################################################Streaming Optimization Function
    '''
       @param DroneController object
       @param int dpid
       @param str occupation'''
    def update_rate(self, dpid, occupation):
        ap_name=self.__dpids[dpid] #name of access point
        ap=self.__datapaths[dpid] #datapath reference of access point

        ofproto=ap.ofproto
        parser=ap.ofproto_parser

        groupH=1 #group id for high quality streaming
        groupL=2 #group rule for low quality streaming

        if 0<=float(occupation)<=1: #avoid incorrect readings
            if self.__occupation[ap_name]!=float(occupation):
                self.__occupation[ap_name]=float(occupation) #update channel occupation

                with open(band_log, "a") as band_file:
                    band_file.write(f"{dt.now()} -> Current Occupation of Bandwidth on access point {ap_name}: {float(occupation)*100}\n")
                    band_file.write(f"\n")

                    if float(occupation)>=0.5: #we assume congestion to happen after 50% bandwidth occupation, we instruct the drone to reduce video quality
                        if self.__congested[ap_name]==0: #if the channel was not previously congested
                            band_file.write(f"{dt.now()} -> Congestion detected on access point {ap_name}. Changing streaming rules\n")
                            band_file.write(f"\n")

                            #########################################Delete Group Rule related to high-quality streaming
                            del_reqH=parser.OFPGroupMod(datapath=ap, command=ofproto.OFPGC_DELETE, group_id=groupH) #remove group for high quality streaming
                            ap.send_msg(del_reqH) #controller sends GroupMod to current AP to clear previous entry for this group

                            with open(log_path, "a") as log_file:
                                log_file.write(f"{dt.now()} -> Congestion detected on access point {ap_name}, {dpid} ({ap})\n")
                                log_file.write(f"   '---> Deleted Group {groupH}\n")

                            ##########################################Delete Flow Rule related to high-quality streaming
                            for ip in self.__ip_groups[ap_name][:2]: #for each drone IPv4 address connected to current AP
                                with open(log_path, "a") as log_file:
                                    log_file.write(f"   '---> Flow rule {ip}:{8888} removed on {ap_name},{ap}\n")

                            #############################################Create New Group Rule for Low Quality Streaming
                            buckets_L=[] #create bucket of action lists (low-quality)
                            for i in range(2, 6): #for each Ethernet interface
                                eth_iface=f'{ap_name}-eth{i}'
                                try:
                                    out_port=self.__access_points[ap_name][eth_iface.encode()] #output port number: Ethernet Interface
                                    action=parser.OFPActionOutput(out_port)
                                    buckets_L.append(parser.OFPBucket(actions=[action]))

                                    with open(log_path, "a") as log_file:
                                        log_file.write(f"   '---> Action: {eth_iface} -> out_port {out_port}\n")

                                except KeyError:
                                    print(f"[ERROR] Ethernet Interface '{eth_iface}' missing on {ap_name}")

                            reqL=parser.OFPGroupMod(datapath=ap,
                                                    command=ofproto.OFPGC_MODIFY,
                                                    type_=ofproto.OFPGT_ALL,
                                                    group_id=groupL,
                                                    buckets=buckets_L) #group mod message
                            ap.send_msg(reqL) #controller issues group mode message to current AP

                            with open(log_path, "a") as log_file:
                                log_file.write(f"   '---> Group rule {groupL} modified on {ap_name}, {ap}\n")
                                log_file.write(f"   '---> Buckets: {buckets_L}\n")
                                log_file.write(f"\n")

                            self.__congested[ap_name]=3 #channel is congested, it has to be found idle 3 times before changing the rules again

                        elif 1<=self.__congested[ap_name]<=2: #the channel is already congested, but starting to get idle
                            self.__congested[ap_name]+=1 #increase the counter again, channel is not getting idle

                    else: #channel occupation below 0.5
                        if 2<=self.__congested[ap_name]<=3: #the channel is congested, but it is getting idle
                            self.__congested[ap_name]-=1 #decrease counter

                        elif self.__congested[ap_name]==1: #channel was congested, but now is idle
                            band_file.write(f"{dt.now()} -> Congestion relieved on access point {ap_name}. Re-establishing streaming rules\n")
                            band_file.write(f"\n")

                            ########################################################Create New Group Rules for Streaming
                            buckets_L=[] #create bucket of action lists (low-quality)
                            buckets_H=[] #create bucket of action lists (high-quality)

                            for i in range(2, 6): #for each Ethernet interface
                                eth_iface=f'{ap_name}-eth{i}'
                                try:
                                    out_port=self.__access_points[ap_name][eth_iface.encode()] #output port number: Ethernet Interface
                                    action=parser.OFPActionOutput(out_port)

                                    if eth_iface[7]=='2': #eth2
                                        if ap_name=='ap1' or ap_name=='ap3':
                                            buckets_H.append(parser.OFPBucket(actions=[action]))
                                        else: #ap2 and ap4
                                            buckets_L.append(parser.OFPBucket(actions=[action]))

                                    elif eth_iface[7]=='3': #eth3
                                        if ap_name=='ap1':
                                            buckets_L.append(parser.OFPBucket(actions=[action]))
                                        else: #ap2, ap3, and ap4
                                            buckets_H.append(parser.OFPBucket(actions=[action]))

                                    elif eth_iface[7]=='4': #eth4
                                        if ap_name=='ap3' or ap_name=='ap4':
                                            buckets_L.append(parser.OFPBucket(actions=[action]))
                                        else: #ap1 and ap2
                                            buckets_H.append(parser.OFPBucket(actions=[action]))

                                    else: #eth5
                                        if ap_name=='ap4':
                                            buckets_H.append(parser.OFPBucket(actions=[action]))
                                        else: #ap1, ap2, and ap3
                                            buckets_L.append(parser.OFPBucket(actions=[action]))

                                    with open(log_path, "a") as log_file:
                                        log_file.write(f"   '---> Action: {eth_iface} -> out_port {out_port}\n")

                                except KeyError:
                                    print(f"[ERROR] Ethernet Interface '{eth_iface}' missing on {ap_name}")

                            reqH=parser.OFPGroupMod(datapath=ap,
                                                    command=ofproto.OFPGC_ADD,
                                                    type_=ofproto.OFPGT_ALL,
                                                    group_id=groupH,
                                                    buckets=buckets_H) #group mod message

                            ap.send_msg(reqH) #controller issues group mode message to current AP
                            with open(log_path, "a") as log_file:
                                log_file.write(f"   '---> Group rule {groupH} re-established on {ap_name}, {ap}\n")
                                log_file.write(f"   '---> Buckets: {buckets_H}\n")

                            reqL=parser.OFPGroupMod(datapath=ap,
                                                    command=ofproto.OFPGC_MODIFY,
                                                    type_=ofproto.OFPGT_ALL,
                                                    group_id=groupL,
                                                    buckets=buckets_L) #group mod message
                            ap.send_msg(reqL) #controller issues group mode message to current AP
                            with open(log_path, "a") as log_file:
                                log_file.write(f"   '---> Group rule {groupL} modified on {ap_name}, {ap}\n")
                                log_file.write(f"   '---> Buckets: {buckets_L}\n")

                            ####################################Re-establish Flow Rule related to high-quality streaming
                            for ip in self.__ip_groups[ap_name][:2]: #for each drone IPv4 address connected to current AP
                                in_port=self.__access_points[ap_name][f'{ap_name}-wlan1'.encode()] #ingress port number: WiFi Interface
                                matchH=parser.OFPMatch(eth_type=0x0800, in_port=in_port, ipv4_src=ip,
                                                       ipv4_dst=self.__broadcastAddress,
                                                       ip_dscp=0b000000, ip_proto=17,
                                                       udp_dst=8888) #match for high quality streams

                                action_udp=parser.OFPActionSetField(udp_dst=int(f"123{ip[10]}"))
                                actions=[action_udp, parser.OFPActionGroup(groupH)] #actions: change destination L4 Port and apply group
                                self.add_flow(datapath=ap, match=matchH, actions=actions, priority=16, meter_id=1) #install flow rule on current AP (high quality)

                                with open(log_path, "a") as log_file:
                                    log_file.write(f"   '---> Flow rule {matchH} re-installed on {ap_name},{ap}\n")
                                    log_file.write(f"   '---> Actions: {actions} \n")

                            with open(log_path, "a") as log_file:
                                log_file.write(f"\n")

                            self.__congested[ap_name]=0 #reset counter

    ###################################################################################################Utility Functions
    '''Install flow rule in the flow table of current OVSAP (the controller sends a FlowMod message to the current AP)
       @param FleetController object
       @param Datapath datapath (reference to current AP)
       @param OFPMatch match
       @param OFPAction actions
       @param int priority (default 0)
       @param int idle_timeout (default 0, meaning no idle timeout)
       @param int hard_timeout (default 0, meaning no hard timeout)
       @param int buffer_id (default None, meaning no buffer ID)
       @param int meter_id (default None, meaning no meter isntruction to apply)'''
    def add_flow(self, datapath, match, actions, priority=0, idle_timeout=0, hard_timeout=0, buffer_id=None, meter_id=None):
        ofproto=datapath.ofproto #to handle OpenFlow protocol 1.3
        parser=datapath.ofproto_parser #to create and manage OpenFlow protocol
        inst=[parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)] #specify to OVSAP instructions to apply actions

        if meter_id is not None:
            inst.append(parser.OFPInstructionMeter(meter_id)) #apply meter instruction if required

            with open(log_path, "a") as log_file:
                log_file.write(f"   '---> Instructions: {inst} \n")
                log_file.write(f"\n")

        if buffer_id: #if a valid buffer ID has been specified by OVS AP to controller
            mod=parser.OFPFlowMod(datapath=datapath, match=match, priority=priority, idle_timeout=idle_timeout, hard_timeout=hard_timeout,
                                    buffer_id=buffer_id, instructions=inst, command=ofproto.OFPFC_ADD) #create FlowMod message
        else: #if NO valid buffer ID has been specified
            mod=parser.OFPFlowMod(datapath=datapath, match=match, priority=priority, idle_timeout=idle_timeout, hard_timeout=hard_timeout,
                                    instructions=inst, command=ofproto.OFPFC_ADD) #create FlowMod message
        datapath.send_msg(mod) #send FlowMod message to OVSAP to install new flow rule

    '''It handles notifications by updating drones' current positions
        @param FleetController object
        @param str drone_mac
        @param str position'''
    def update_pos(self, drone_mac, position):
        pos=tuple(map(float, position.split(',')))
        drone=self.__drone_macs[drone_mac]

        if pos!=self.__drone_positions[drone]: #if drone position has changed

            self.__drone_positions[drone]=pos #update current drone position

            with open(mob_log, "a") as log_file:
                log_file.write(f"{dt.now()} -> Receiving position update from {drone}: {drone_mac}\n")
                log_file.write(f"   '---> Current position: {pos}\n")
                log_file.write(f"\n")

                for d,p in self.__drone_positions.items():
                    if p==self.__hover_positions[d]:
                        status='Deployed'
                    elif p==self.__base:
                        status='Base'
                    else:
                        status='Moving'

                    log_file.write(f"   '---> {d}:{p}. Status: {status}. Remaining energy: {self.__drone_energy[d]} [J]\n")

                log_file.write(f"\n")

    '''Send mobility command to drone over a socket connection, instructing it to go back to base (if the drone's battery is below the recharge threshold)
       or to start to hover and stream video (if the drone has been spared)
       @param str command'''
    def send_command(self, command):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.__address, self.__port)) #connects to position server socket
                s.sendall(command.encode()) #send moving drone command
                ack=s.recv(1024) #wait for an ACK

                jcommand = json.loads(command) #convert transmitted command in json format
                with open(mob_log, "a") as log_file:
                    log_file.write(f"{dt.now()} -> [ACK] Received: {ack.decode()} to {jcommand['Drone']} ")
                    log_file.write("\n")
        except Exception as e:
            print(f"[ERROR] Failed to Send Command: {e}")

    '''Convert a DPID in its hexadecimal format
       @param int dpid
       @return str dpid'''
    @staticmethod
    def dpid_to_hex(dpid: int) -> str:
        if dpid<0:
            raise ValueError("DPID must be a non-negative integer.")
        return f"{dpid:016x}"
