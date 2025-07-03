#!/usr/bin/env python3

"""Usage:
   sudo python3 FootballStreaming.py [--plot] Optional: use plotGraph function to visualize the network of drones and APs in a 2D space. The plot will show
                                                        the movement of drones in real time
                                     [--ap_tel/--dr_tel] Optional: use telemetry class to show wireless channel metrics across time measured on either the APs or
                                                                      the drones. Since the plot is unique, you can specify either --plot or --aps_tel/--dr_tel
                                     [--rssi/--tx_bytes/--tx_packets/--tx_errors/--tx_dropped/--tx_fifo_errors/--collisions (drone metrics)
                                      --rx_bytes/--rx_packets/--rx_errors/--rx_missed_errors/--rx_crc_errors/--rx_dropped/
                                      --rx_over errors/--rx_fifo_errors (AP metrics)] Optional: if either --ap_tel or --dr_tel has been specified, a metric to
                                                                                      observed has to be specified also. Since the plot is unique, a single metric
                                                                                      can be shown at a time
                                     [--bgscan] Optional: handle AP <-> drone association through background scanning. When using background scanning, all AP
                                                          will have the same SSID, allowing drones to roam. Otherwise, each AP has a dedicated SSID
                                     [--manual] Optional: handle AP <-> drone association by explicitly forcing associations with setAssociation function. If
                                                          --manual is not specified, auto-association of drones to APs is enabled
                                     [--arp] Optional: pre-computes ARP tables statically in all endpoints and drones
                                     [--set_params] Optional: set tx power, antenna gain, and supported data rate in drones and APs
                                     [--endpoint1/--endpoint2/--endpoint3/--endpoint4] Optional: receive video streams only on a specific endpoints. If an
                                                                                                 endpoint is not specified, video streams will be received
                                                                                                 on all endpoints at the same time
                                     [--drone1/--drone2/--drone3/--drone4] Optional: stream video only from a specific drone. If a drone is not specified, video
                                                                                     streams will be transmitted from all drones at the same time
                                     [--test] Optional: generate congestion on first WiFi Channel to test streaming adaptability"""

#################################################################################################################Imports
from mn_wifi.net import Mininet_wifi
from mininet.log import setLogLevel, info
from mn_wifi.cli import CLI

from mininet.node import RemoteController
from mn_wifi.node import OVSKernelAP
from mininet.link import TCLink
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference
from mn_wifi.replaying import ReplayingMobility
from mn_wifi.telemetry import telemetry

import sys
import subprocess
import re
import threading
import time
from datetime import datetime as dt
import socket
import json

############################################################################################Network Deployment Functions
'''Creates a Mininet_wifi object.
   Deploys drones (Station objects), APs (AccessPoint objects), and endpoints (Host objects).
   Establishes AP-endpoint links and defines WiFi channel features
   @param str[] args'''
def topology(args):
    global net

    net=Mininet_wifi(topo=None, build=False,
                     controller=RemoteController,
                     link=wmediumd,
                     wmediumd_mode=interference,
                     ac_method='ssf', #drone<->AP association control: strongest signal first
                     autoAssociation=True, allAutoAssociation=True, #if True, Mininet WiFi automatically handles drone<->AP association
                     mode="a", freq=5,
                     ipBase="192.168.1.0/24")

    net.client_isolation=True #drones cannot communicate without an explicit OpenFlow rule, even if associated to the same AP

    if '--bgscan' in args:
        net.ssid="FootStream" #for background scanning to work, APs must have the same SSID

    if '--manual' in args: #disable automatic association control
        net.autoAssociation=False
        net.allAutoAssociation=False

    info("************Creating Network Nodes**********\n")

    drones=[] #list of all drones in the network
    for i in range(1, 9):
        drones.append(net.addStation(name=f'drone{i}', ip=f'192.168.1.{i}/24',
                                     speed=1, #for drones mobility (must be an integer)
                                     position='0,0,0'))

        if '--bgscan' in args: #configure background scan on drones
            drones[i-1].params['bgscan_threshold']=-65 #watch out for this value (must not be greater than the received RSSI)
            drones[i-1].params['s_interval']=10
            drones[i-1].params['l_interval']=30
            drones[i-1].params['bgscan_module']='simple'

    if '--test' in args:
        net.addStation(name=f'station1', ip=f'192.168.1.21/24', position=f'4,4,0')

    endpoints=[] #list of all client endpoints in the network
    for i in range(9, 13):
        endpoints.append(net.addHost(name=f'endpoint{i-9+1}', mac=f"00:00:00:00:00:{i:02x}", ip=f'192.168.1.{i}/24'))

    if '--test' in args:
        host2=net.addHost(name='host2', ip=f'192.168.1.22/24')

    #adding 4 APs
    if '--bgscan' not in args:
        ap1=net.addAccessPoint(name='ap1', dpid=f"{13:016x}", ssid='ssid-ap1',
                               cls=OVSKernelAP, protocols='OpenFlow13', listenPort=6654,
                               channel='36',
                               position='0,0,0') #bottom-left corner AP
        ap2=net.addAccessPoint(name='ap2', dpid=f"{14:016x}", ssid='ssid-ap2',
                               cls=OVSKernelAP, protocols='OpenFlow13', listenPort=6655,
                               channel='40',
                               position='105,0,0') #bottom-right corner AP
        ap3=net.addAccessPoint(name='ap3', dpid=f"{15:016x}", ssid='ssid-ap3',
                               cls=OVSKernelAP, protocols='OpenFlow13', listenPort=6656,
                               channel='44',
                               position='105,68,0') #top-right corner AP
        ap4=net.addAccessPoint(name='ap4', dpid=f"{16:016x}", ssid='ssid-ap4',
                               cls=OVSKernelAP, protocols='OpenFlow13', listenPort=6657,
                               channel='48',
                               position='0,68,0') #top-left corner AP
    else:
        ap1=net.addAccessPoint(name='ap1', dpid=f"{13:016x}",
                               cls=OVSKernelAP, protocols='OpenFlow13', listenPort=6654,
                               channel='36',
                               position='0,0,0') #bottom-left corner AP
        ap2=net.addAccessPoint(name='ap2', dpid=f"{14:016x}",
                               cls=OVSKernelAP, protocols='OpenFlow13', listenPort=6655,
                               channel='40',
                               position='105,0,0') #bottom-right corner AP
        ap3=net.addAccessPoint(name='ap3', dpid=f"{15:016x}",
                               cls=OVSKernelAP, protocols='OpenFlow13', listenPort=6656,
                               channel='44',
                               position='105,68,0') #top-right corner AP
        ap4=net.addAccessPoint(name='ap4', dpid=f"{16:016x}",
                               cls=OVSKernelAP, protocols='OpenFlow13', listenPort=6657,
                               channel='48',
                               position='0,68,0') #top-left corner AP

    info(f"\n**********Wifi Configuration**********\n")
    net.setPropagationModel(model="logDistance", exp=2.5) ##############################################################
    net.configureWifiNodes() #create and configure L2 wireless interfaces on drones and APs

    #establishing Ethernet connections between APs and endpoints (must follow the call to configureWifiNodes to ensure consistent interface naming)
    net.addLink(node1=ap1, node2=endpoints[0], port1=2, port2=1, cls=TCLink, bw=1000)
    net.addLink(node1=ap2, node2=endpoints[1], port1=2, port2=1, cls=TCLink, bw=1000)
    net.addLink(node1=ap3, node2=endpoints[2], port1=2, port2=1, cls=TCLink, bw=1000)
    net.addLink(node1=ap4, node2=endpoints[3], port1=2, port2=1, cls=TCLink, bw=1000)

    #establishing backbone Ethernet connections between APs, forming a mesh pattern
    net.addLink(node1=ap1, node2=ap2, port1=3, port2=3, cls=TCLink, bw=1000)
    net.addLink(node1=ap1, node2=ap3, port1=4, port2=3, cls=TCLink, bw=1000)
    net.addLink(node1=ap1, node2=ap4, port1=5, port2=3, cls=TCLink, bw=1000)

    net.addLink(node1=ap2, node2=ap3, port1=4, port2=4, cls=TCLink, bw=1000)
    net.addLink(node1=ap2, node2=ap4, port1=5, port2=4, cls=TCLink, bw=1000)

    net.addLink(node1=ap3, node2=ap4, port1=5, port2=5, cls=TCLink, bw=1000)

    if '--test' in args:
        net.addLink(node1=net.getNodeByName('ap1'), node2=host2, port1=6, port2=1, cls=TCLink, bw=1000)

    if '--plot' in args:
        net.plotGraph(min_x=0, min_y=0, max_x=110, max_y=110) #visual representation of drones fleet and APs on a 2D space

'''Manually associate drones to APs using setAssociation function'''''
def drone_association():
    global net

    print("\n")
    print("**********Associating Drones and APs**********\n")

    for drone in net.stations: #Force specific wireless associations between drones and APs
        if drone.name=='drone1' or drone.name=='drone5' or drone.name=='station1':
            ap=net.getNodeByName('ap1')

        elif drone.name=='drone2' or drone.name=='drone6':
            ap=net.getNodeByName('ap2')

        elif drone.name=='drone3' or drone.name=='drone7':
            ap=net.getNodeByName('ap3')

        else: #drone4 and drone8
            ap=net.getNodeByName('ap4')

        print(f"Drone {drone.name}, {drone} associating with {ap.name}, {ap}\n")
        drone.setAssociation(ap)

'''Starting network emulation
   @param str[] args'''
def net_start(args):
    global net

    info(f"\n**********Network Emulation is Starting**********\n")

    controller=net.addController('controller', controller=RemoteController, ip='127.0.0.1', port=6653) #add remote controller to the network
    net.build()
    controller.start()

    for ap in net.aps:
        net.get(ap.name).start([controller]) #starts APs and force connection to remote controller listening on localhost:6653

    for station in net.stations: #apply sysctl settings to all drones (station devices)
        station.cmd('sysctl -p') #disable ignore_icmp_broadcast on drones

    for endpoint in net.hosts: #apply sysctl settings to all endpoints (host devices)
        endpoint.cmd('sysctl -p') #disable ignore_icmp_broadcast on drones

    if '--arp' in args:
        net.staticArp() #pre-computes ARP tables statically

'''Manually setting radio parameters in APs and drones (tx power, tx rate, and antenna gain)'''
def set_params():
    global net

    for drone in net.stations:
        intf_name=drone.wintfs[0]

        drone.setAntennaGain(5, intf=intf_name) ########################################################################
        drone.params['txrate']='54Mbps'
        #drone.cmd(f'iw dev {intf_name} set bitrates legacy-5 48 54')

    for ap in net.aps:
        intf_name=ap.wintfs[0]

        ap.setAntennaGain(10, intf=intf_name) ###########################################################################
        ap.params['txrate']='54Mbps'
        #ap.cmd(f'iw dev {intf_name} set bitrates legacy-5 48 54')

########################################################################Network Mobility and Energy Management Functions
'''Handles all movements of drones'''
def mobility():
    global net

    print("\n**********Deploying drones fleet**********\n")

    net.isReplaying=True

    path='/home/francesco2/Documenti/PycharmProjects/Smart2/Mobility Traces/' #path containing the mobility patterns (sequence of positions in time) of drones

    get_trace(net.getNodeByName('drone1'), '{}pos1.dat'.format(path)) #each mobility pattern is a .dat file
    get_trace(net.getNodeByName('drone2'), '{}pos2.dat'.format(path))
    get_trace(net.getNodeByName('drone3'), '{}pos3.dat'.format(path))
    get_trace(net.getNodeByName('drone4'), '{}pos4.dat'.format(path))
    get_trace(net.getNodeByName('drone5'), '{}pos5.dat'.format(path))
    get_trace(net.getNodeByName('drone6'), '{}pos6.dat'.format(path))
    get_trace(net.getNodeByName('drone7'), '{}pos7.dat'.format(path))
    get_trace(net.getNodeByName('drone8'), '{}pos8.dat'.format(path))

'''Given a drone (Station object) and the path of the .dat file containing its mobility pattern, it extracts the sequence of positions and assigns it to an
   an attribute p of the Station object
   @param Station drone
   @param str file_'''
def get_trace(drone, file_):
    file_=open(file_, 'r') #open .dat file
    raw_data=file_.readlines() #extract mobility pattern line by line
    file_.close()

    drone.p=[] #initialize attribute p (list of positions)
    pos=(-1000, 0, 0)
    drone.position=pos #initialize position attribute of drone

    for data in raw_data: #for each line
        line=data.split()

        x=line[0] #First Column (X coordinate)
        y=line[1] #Second Column (Y coordinate)
        z=line[2] #Third Column (Z coordinate)
        pos=float(x), float(y), float(z) #create position as 3-tuple (float, float, float)
        drone.p.append(pos) #save position into list

'''This function defines a mobility pattern (sequence of position occupied in time) for each drone of the fleet, according to a pre-determined energy model
   @param <str: tuple> positions'''
def energy_model(positions):
    ###################################################################################################Drones parameters
    #consdering drone DJI Mavic 3E
    capacity=5000 #Battery capacity [mAh]
    voltage=15.4 #Battery voltage [V]
    gravity=9.8 #Gravitational acceleration [m/s^2]

    e_battery=capacity*voltage*3.6 #Full-battery energy [J]

    m1=1 #Drone body mass [kg]
    m2=0.3 #Drone battery mass [kg]
    m3=0.1 #Drone camera mass [kg]
    m=m1+m2+m3 #[kg]

    lift_to_drag=2.5 #Lift-to-drag ratio: typical range of aerodynamic efficiency for multi-rotor drones is 1-3
    efficiency=0.5 #Rotors efficiency: typical range is 0.2-0.7

    p_avio=10 #Power needed for avionics operations (on-board computations, communications, video-recording, gps, etc) [W]: 5-25 [W] of typical range
    v=5 #Drone's speed relative to the wind [m/s]: 5-15 [m/s] is a typical speed range for video drones
    phi=0.3 #windspeed-to-dronespeed ratio: typical range 0-0.6

    n=4 #number of rotors
    s=0.073 #rotor's area [m^2]: 0.049-0.113 [m^2] is a typical range for quad-rotor drones
    ro=1.152062 #air density in Cosenza
    drag_coeff=1.3 #vertical drag coefficient: 1-1.5 is a typical range for quad-rotor drones

    ########################################################################################################Computations
    for drone in positions.keys():
        print(f"Drone {drone}, target hovering position: {positions[drone]}, available energy: {e_battery} [J]")

        #First movement: vertical lift (along Z-axis). Assumptions: constant speed, thrust == weight + drag, zero lift
        height=positions[drone][2] #[m] Along Z-Axis
        e_lift= ( ( (m*gravity) + ( n*ro*s*drag_coeff*(v**2) )/2 ) * (v/efficiency) + p_avio ) * (height/v) #[J] energy for first movement
        print(f"    '---> {drone} first movement: (0,0,0) -> (0,0,{height}). Consumed energy: {e_lift} [J]")

        #Second movement: horizontal flight along X-axis. Assumptions: constant speed, thrust == drag, weight == lift
        distance1=positions[drone][0] #[m] Along X-axis
        e_fly1=( ( (m*gravity) / (efficiency*lift_to_drag) ) + (p_avio/ (v/(1-phi)) ) ) * (distance1 / (1-phi) ) #[J] energy for second movement
        print(f"    '---> {drone} second movement: (0,0,{height}) -> ({distance1},0,{height}). Consumed energy: {e_fly1} [J]")

        #Third movement: horizontal flight along Y-axis. Assumptions: constant speed, thrust == drag, weight == lift
        distance2=positions[drone][1] #[m] Along Y-axis
        e_fly2=( ( (m*gravity) / (efficiency*lift_to_drag) ) + (p_avio/ (v/(1-phi)) ) ) * (distance2 / (1 - phi)) #[J] energy for third movement
        print(f"    '---> {drone} third movement: ({distance1},0,{height}) -> ({distance1},{distance2},{height}). Consumed energy: {e_fly2} [J]")

        #Fourth movement: like third
        e_fly3=e_fly2 #[J] energy for fourth movement
        print(f"    '---> {drone} fourth movement (after hovering): ({distance1},{distance2},{height}) -> ({distance1},0,{height}). Consumed energy: {e_fly3} [J]")

        #Fifth movement: like second
        e_fly4=e_fly1 #[J] energy for fifth movement
        print(f"    '---> {drone} fifth movement (after hovering): ({distance1},0,{height}) -> (0,0,{height}). Consumed energy: {e_fly4} [J]")

        #Last movement: vertical drop (along Z-axis). Assumptions: constant speed, thrust + drag == weight, zero lift
        e_drop=( ( (m*gravity) + ( n*ro*s*drag_coeff*(v**2) ) /2 ) * (v/efficiency) + p_avio ) * (height/v) #[J] energy for first movement
        print(f"    '---> {drone} sixth movement (after hovering): (0,0,{height}) -> (0,0,0). Consumed energy: {e_drop} [J]")

        #Hovering at fixed altitude. Assumptions: zero speed, thrust == weight, zero drag, zero lift
        e_hover=e_battery-e_lift-e_fly1-e_fly2-e_fly3-e_fly4-e_drop #[J] maximum energy available for hovering
        t_h=e_hover / ( ( (m*gravity)**(3/2) / (2*n*ro*s)**(1/2) ) + p_avio ) #[s] maximum hovering time
        print(f"    '---> {drone} maximum available hovering energy: {e_hover} [J]. Maximum hovering time: {t_h} [s]")

        total_time=round(height/v)+round(distance1/v)+round(distance2/v)+round(t_h/20)+round(distance1/v)+round(distance2/v)+round(height/v) #total flight time [s]
        print(f"    '---> {round(height/v)}+{round(distance1/v)}+{round(distance2/v)}+{round(t_h/20)}+{round(distance2/v)}+{round(distance1/v)}"
              f"+{round(height/v)} [s]. Total = {total_time} [s]\n")

        write_dat(drone, height, distance1, distance2, v, round(t_h/20)) #compute mobility pattern according to energy consumption model

'''Write the .dat file with the mobility pattern for a given drone (it contains the series of positions occupied by the drone during the simulation).
   @param str drone
   @param int height
   @param int distance_x
   @param int distance_y
   @param float v
   @param int t_h'''
def write_dat(drone, height, distance_x, distance_y, v, t_h):
    numb=int(re.search(r'\d+', drone).group()) #extract drone number
    dat_name=f"pos{numb}.dat"
    path='/home/francesco2/Documenti/PycharmProjects/Smart2/Mobility Traces/'+f'{dat_name}'
    print(f"{drone} - Mobility Pattern in {path}")

    with open(path, "w") as log_file:
        log_file.write(f"0 0 0\n") #write first position into .dat file

    lift_time=round(height/v) #[s] time required to reach the height (it must be an integer number of seconds)
    step=height/lift_time #[m] distance covered in each second of movement
    lift_pos=[] #intermediate positions during lift
    for i in range(1, lift_time+1): #for the time required to lift to height
        lift_pos.append(0+i*step) #each second corresponds to an occupied position

    fly_x_time=round(distance_x/v) #[s] time required to reach the X position (it must be an integer number of seconds)
    step=distance_x/fly_x_time #[m] distance covered in each second of movement
    x_pos=[] #intermediate positions during horizontal flight along X axis
    for i in range(1, fly_x_time+1): #for the time required to reach X position
        x_pos.append(0+i*step) #each second corresponds to an occupied position

    fly_y_time=round(distance_y/v) #[s] time required to reach the Y position (it must be an integer number of seconds)
    step=distance_y/fly_y_time #[m] distance covered in each second of movement
    y_pos=[] #intermediate positions during horizontal flight along Y axis
    for i in range(1, fly_y_time+1): #for the time required to reach Y position
        y_pos.append(0+i*step) #each second corresponds to an occupied position

    y_pos_inv=y_pos[-2::-1] #retracing steps to come back to Y=0
    y_pos_inv.append(0)

    x_pos_inv=x_pos[-2::-1] #retracing steps to come back to X=0
    x_pos_inv.append(0)

    drop_pos=lift_pos[-2::-1] #retracing steps to come back to Z=0
    drop_pos.append(0)

    print(f"    '---> {lift_pos}")
    print(f"    '---> {drop_pos}")
    print(f"    '---> {x_pos}")
    print(f"    '---> {x_pos_inv}")
    print(f"    '---> {y_pos}")
    print(f"    '---> {y_pos_inv}\n")

    with open(path, "w") as log_file:
        if numb in range(1, 5): #for drones 1 to 4 (immediately deployed)
            for p in lift_pos:
                log_file.write(f"0 0 {p}\n") #write lifting positions

            for p in x_pos:
                log_file.write(f"{p} 0 {height}\n") #write flight positions along X axis

            for p in y_pos:
                log_file.write(f"{distance_x} {p} {height}\n") #write flight positions along Y axis

            for t in range(t_h):
                log_file.write(f"{distance_x} {distance_y} {height}\n") #repeat hovering position for hovering time

            for p in y_pos_inv:
                log_file.write(f"{distance_x} {p} {height}\n") #write backward flight positions along Y axis

            for p in x_pos_inv:
                log_file.write(f"{p} 0 {height}\n") #write backward flight positions along X axis

            for p in drop_pos:
                log_file.write(f"0 0 {p}\n") #write descend positions

        else: #for drones 5 to 8 (spare drones)
            for t in range(t_h):
                log_file.write(f"0 0 0\n") #stay at supply station for hovering time

            for p in lift_pos:
                log_file.write(f"0 0 {p}\n") #write lifting positions

            for p in x_pos:
                log_file.write(f"{p} 0 {height}\n") #write flight positions along X axis

            for p in y_pos:
                log_file.write(f"{distance_x} {p} {height}\n") #write flight positions along Y axis

'''Thread target function. Server Socket listening for mobility commands from the controller'''
def start_server():
    global net, HOST, PORT, log_path, stop_event

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.bind((HOST, PORT))
        server_sock.listen() #open server socket

        with open(log_path, "w") as log_file:
            log_file.write(f"{dt.now()} -> [READY] Server listening on {HOST}:{PORT}...\n")
            log_file.write(f"\n")

        while not stop_event.is_set(): #for the whole simulation (while True)
            conn, addr=server_sock.accept() #wait for incoming connections

            with conn: #if connection is established
                with open(log_path, "a") as log_file:
                    log_file.write(f"{dt.now()} -> [CONNECTED] Connection from {addr}\n")

                    data=conn.recv(1024)
                    if data: #if data is received
                        command=data.decode() #obtain command from received data
                        jcommand=json.loads(command) #convert received command in json format

                        log_file.write(f"   '---> Received command: {command}\n")
                        log_file.write(f"   '---> Drone: {jcommand['Drone']}\n")
                        log_file.write(f"   '---> Action: {jcommand['Action']}\n")
                        log_file.write(f"   '---> Start Position: {jcommand['Start Position']}\n")
                        log_file.write(f"   '---> Final Position: {jcommand['Final Position']}\n")
                        log_file.write(f"\n")

                    conn.sendall(b"ACK") #send ACK to signal correct command reception

#########################################################################################Network Transmissions Functions
'''Thread target function. Given a source drone, it periodically sends a notification of its current position
   @param str drone_name
   @param str drone_ipv4
   @param str broadcast_address'''
def send_position(drone_name, drone_ipv4, broadcast_address):
    global net, stop_event

    drone=net.getNodeByName(drone_name) #Mininet-emulated source Drone (Station reference)

    time.sleep(20) #initial waiting time

    pos=','.join(map(str, drone.position)) #current drone position (converted from tuple to string)
    rx_bytes=0 #byte received on AP wlan interface

    #print(f"Drone {drone_name} ({drone_ipv4}). Current position: {pos} {type(pos)}\n")
    drone.popen(f'sudo python3 /home/francesco2/Documenti/PycharmProjects/Smart2/SendPosition.py '
              f'--drone={drone_name} --src={drone_ipv4} --dst={broadcast_address} --pos={pos} ' #send notification
              f'> /home/francesco2/Documenti/PycharmProjects/Smart2/logFiles/{drone_name}_notification.log 2>&1 &', shell=True) #record notification on a log file

    t=20 #[s] time interval in between position notification

    while not stop_event.is_set(): #for the whole simulation
        time.sleep(t) #waiting interval between notifications

        if pos=='0.0,0.0,0.0' and pos==','.join(map(str, drone.position)): #if the previous position was (0.0, 0.0, 0.0) and has not changed
            continue #do nothing
        else:
            pos=','.join(map(str, drone.position)) #update current drone position (converted from tuple to string)

            numb=int(drone_name[5]) #drone number
            if numb<=4: #drone 1-4
                ap_name=f'ap{numb}' #obtain AP name from drone's number
            else: #drone 5-8
                ap_name=f'ap{numb-4}' #obtain AP name from drone's number

            with open(f'/sys/class/net/{ap_name}-wlan1/statistics/rx_bytes', 'r') as f:
                cur_rx_bytes=int(f.read().strip()) #obtain count of received bytes of AP's wlan interface

            proc=drone.popen(f'iw dev {drone_name}-wlan0 link', stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True) #read drone's WiFi stats
            result_b, error_b=proc.communicate() #command output (bytes)
            result=result_b.decode('utf-8') #command output (string)

            match=re.search(r'tx bitrate:\s+([0-9.]+)\s+MBit/s', result) #search for current allowed transmission rate
            tx_bitrate=1
            if match:
                tx_bitrate=float(match.group(1)) #fetch tx rate

            occupation=( ( (cur_rx_bytes-rx_bytes)*8 ) / (t* (10**6)) ) / tx_bitrate #compute channel bandwidth occupation from byte count and tx rate

            rx_bytes=cur_rx_bytes #update received bytes count

            #print(f"Drone {drone_name} ({drone_ipv4}). Current position: {pos} {type(pos)}. Available bandwidth: {tx_bitrate}. Occupation: {occupation}\n")
            drone.popen(f'sudo python3 /home/francesco2/Documenti/PycharmProjects/Smart2/SendPosition.py '
                      f'--drone={drone_name} --src={drone_ipv4} --dst={broadcast_address} --pos={pos} --occ={occupation}' #send notification
                      f'> /home/francesco2/Documenti/PycharmProjects/Smart2/logFiles/{drone_name}_notification.log 2>&1 &', shell=True) #record notifications

'''Thread target function. Start/close video streaming from drones, also start/stop streaming reception on endpoints
   @param str drone_name
   @param str videoH (high quality file video)
   @param str videoL (low quality file video)
   @param str broadcast_addr
   @param int portH
   @param int portL
   @param str[] args'''
def send_video(drone_name, videoH, videoL, broadcast_addr, portH, portL, args):
    global net, drone_positions, base_pos, rec_streams, stream_dst_port, stop_event

    drone=net.getNodeByName(drone_name) #Mininet-emulated source Drone (Station reference)
    pos=drone.position #current drone position (tuple)

    procH=None #initialize process object
    procL=None #initialize process object
    if pos!=base_pos: #if the drone is in not at the supply base
        #print(f'{drone.name} transmitting video\n')
        procH=drone.popen(f"vlc-wrapper {videoH} --sout '#udp{{dst={broadcast_addr}:{portH}}}' :no-sout-all :sout-keep", shell=True) #stream video process
        procL=drone.popen(f"vlc-wrapper {videoL} --sout '#udp{{dst={broadcast_addr}:{portL}}}' :no-sout-all :sout-keep", shell=True) #stream video process

    t=20 #[s] time interval in between position notification

    while not stop_event.is_set(): #for the whole simulation
        time.sleep(t) #waiting interval between notifications

        if drone.position!=base_pos and pos==base_pos: #if the drone has started to move
            pos=drone.position
            #print(f'drone {drone_name} has moved\n')
            #print(f'{drone.name} transmitting video\n')

            if drone_name=='drone5': #force drone5 to connect to AP1 when deployed
                time.sleep(2)
                drone.popen(f'iw dev {drone_name}-wlan0 disconnect', shell=True)
                time.sleep(2)
                drone.popen(f'iw dev {drone_name}-wlan0 connect ssid-ap1', shell=True)
                #ap=net.getNodeByName('ap1')
                #drone.setAssociation(ap)
                #drones[i].popen(f'iw dev drone{i+1}-wlan0 set bitrates legacy-5 12 54', shell=True)

            procH=drone.popen(f"vlc-wrapper {videoH} --sout '#udp{{dst={broadcast_addr}:{portH}}}' :no-sout-all :sout-keep", shell=True) #stream from drone
            procL=drone.popen(f"vlc-wrapper {videoL} --sout '#udp{{dst={broadcast_addr}:{portL}}}' :no-sout-all :sout-keep", shell=True) #stream from drone

            if any('--endpoint' in arg for arg in args):
                endpoint_name=next((arg[2:] for arg in args if '--endpoint' in arg), None)
                endpoint=net.getNodeByName(endpoint_name) #receive only on a specific endpoint
                k=stream_dst_port+int(drone_name[5])-1 #L4 destination port where to receive from the current drone

                #print(f'{endpoint.name} waiting for video on {endpoint.IP()}:{k}\n')
                rec_proc=endpoint.popen(f'vlc-wrapper udp://@{broadcast_address}:{k}', shell=True) #receive from current drone on current endpoint
                rec_streams[endpoint.name][k]=rec_proc

            else: #receive on all endpoints
                for endpoint in net.hosts:
                    k=stream_dst_port+int(drone_name[5])-1 #L4 destination port where to receive from the current drone

                    #print(f'{endpoint.name} waiting for video on {endpoint.IP()}:{k}\n')
                    rec_proc=endpoint.popen(f'vlc-wrapper udp://@{broadcast_address}:{k}', shell=True) ##receive from current drone on all endpoints
                    rec_streams[endpoint.name][k]=rec_proc

            #print(rec_streams)

        elif drone.position!=base_pos and pos!=base_pos: #if the drone is moving/hovering
            pos=drone.position
            continue #do nothing

        elif drone.position==base_pos and pos!=base_pos: #if the drone has come back to base
            pos=drone.position
            procH.terminate() #stop streaming (high quality)
            procL.terminate() #stop streaming (low quality)

            if any('--endpoint' in arg for arg in args):
                endpoint_name=next((arg[2:] for arg in sys.argv if '--endpoint' in arg), None)
                endpoint=net.getNodeByName(endpoint_name)

                k=stream_dst_port+int(drone_name[5])-1
                rec_proc=rec_streams[endpoint.name][k]
                rec_proc.terminate() #stop receiver process associated to drone on designated endpoint

            else:
                for endpoint in net.hosts:
                    k=stream_dst_port+int(drone_name[5])-1

                    rec_proc=rec_streams[endpoint.name][k]
                    rec_proc.terminate() #stop receiver process associated to drone on all endpoints

        else: #drone.position==base_pos and pos==base_pos:
            pos=drone.position
            continue #do nothing

####################################################################################################################Main
if __name__ == '__main__':
    setLogLevel('info')

    drone_positions = {
                     'drone1': (26.25, 17, 35),
                     'drone2': (78.75, 17, 35),
                     'drone3': (78.75, 51, 35),
                     'drone4': (26.25, 51, 35),
                     'drone5': (26.25, 17, 35), #same as drone1
                     'drone6': (78.75, 17, 35), #same as drone2
                     'drone7': (78.75, 51, 35), #same as drone3
                     'drone8': (26.25, 51, 35), #same as drone4
                } #dictionary associating each drone of the fleet to its designated hovering position

    base_pos=(0.0, 0.0, 0.0) #drone supply base position

    HOST='localhost' #IPv4 address of server socket where mobility commands are received from the controller
    PORT=8080 #L4 port number of server socket where mobility commands are received from the controller
    log_path="/home/francesco2/Documenti/PycharmProjects/Smart2/logFiles/CommandsLog.txt" #file where controller mobility commands are memorized

    broadcast_address='192.168.1.255' #drone network broadcast address
    stream_src_portH=8888 #L4 port where drones stream the video (high quality)
    stream_src_portL=7777 #L4 port where drones stream the video (low quality)
    stream_dst_port=1231 #L4 base port where endpoints receive the video

    #video_pathH='high_quality.mp4' #high quality video file
    #video_pathL='low_quality.mp4' #low quality video file
    video_pathH='OnePiece_Ep_0223_SUB_ITA.mp4' #high quality video file
    video_pathL='low_res.mp4' #low quality video file

    print(f'Running Configurations: {sys.argv}\n')

    energy_model(drone_positions) #function defining mobility patterns for each drone according to an energy model
    topology(sys.argv) #function creating the network

    if '--manual' in sys.argv:
        drone_association() #manually associate drones to APs

    mobility() #function handling the mobility of the drones fleet

    threads=[] #array of all threads started during simulation
    stop_event=threading.Event() #this event will be used to signal the threads to stop

    try:
        t=threading.Thread(target=start_server) #starts server socket where mobility commands from the controller are received
        t.start() #start thread running server socket

    except Exception as e:
        print(e)

    try: #starting send position threads
        for drone in net.stations: #for each drone (Station reference)
            if drone.name!='station1':
                t=threading.Thread(target=send_position, args=(drone.name, drone.IP(), broadcast_address)) #create thread sending drone position notifications
                t.start() #start sending thread
                threads.append(t)

    except Exception as e:
        print(e)

    #Dictionary associating each endpoint with receiving processes, identified by their L4 port number
    rec_streams = { 'endpoint1':{},
                    'endpoint2':{},
                    'endpoint3':{},
                    'endpoint4':{} } #dictionary <str endpoint name, <int port number, receiving streaming process>

    if any('--endpoint' in arg for arg in sys.argv):
        endpoint_name=next((arg[2:] for arg in sys.argv if '--endpoint' in arg), None)

        endpoint=net.getNodeByName(endpoint_name) #receive only on a given endpoint

        for i in range(4):
            port_number=stream_dst_port+i
            print(f'{endpoint.name} waiting for video on {endpoint.IP()}:{port_number}\n')

            proc=endpoint.popen(f'vlc-wrapper udp://@{broadcast_address}:{port_number}', shell=True) #receive on 4 different ports on the designated endpoint
            rec_streams[endpoint_name][port_number]=proc
        print(f"\n")

        print(rec_streams)
        print(f"\n")

    else:
        for endpoint in net.hosts: #receive on all endpoints
            if endpoint.name=='host2':
                continue

            if any('--drone' in arg for arg in sys.argv): #if we transmit only from a given drone
                drone_name=next((arg[2:] for arg in sys.argv if '--drone' in arg), None)

                port_number=stream_dst_port+int(drone_name[5])-1
                print(f'{endpoint.name} waiting for video on {endpoint.IP()}:{port_number}\n')

                proc=endpoint.popen(f'vlc-wrapper udp://@{broadcast_address}:{port_number}', shell=True) #receive only on a given port on each endpoint
                rec_streams[endpoint.name][port_number]=proc

            else: #if we transmit from all deployed drones at the same time
                for i in range(4):
                    port_number=stream_dst_port+i
                    print(f'{endpoint.name} waiting for video on {endpoint.IP()}:{port_number}\n')

                    proc=endpoint.popen(f'vlc-wrapper udp://@{broadcast_address}:{port_number}', shell=True) #receive on 4 different ports on each endpoint
                    rec_streams[endpoint.name][port_number]=proc
                print(f"\n")

        print(rec_streams)
        print(f"\n")

    metrics=['rssi', 'tx_bytes', 'tx_packets', 'tx_errors', 'tx_dropped', 'tx_fifo_errors', 'collisions',
             'rx_bytes', 'rx_packets', 'rx_errors', 'rx_missed_errors', 'rx_crc_errors', 'rx_dropped', 'rx_over errors', 'rx_fifo_errors'] #observable telemetries

    if '--ap_tel' in sys.argv:
        for m in metrics:
            if f'--{m}' in sys.argv:
                telemetry(nodes=net.aps, single=True, data_type=m)
                break

    if '--dr_tel' in sys.argv:
        for m in metrics:
            if f'--{m}' in sys.argv:
                telemetry(nodes=net.stations, single=True, data_type=m)
                break

    net_start(sys.argv) #function kick-starting network emulation

    if '--set_params' in sys.argv:
        set_params()

    info(f"\n**********Replaying Mobility**********\n")
    ReplayingMobility(net) #function executing the mobility pattern for each drone

    time.sleep(20) #wait some time before starting to transmit

    try: #starting video streaming threads
        if any('--drone' in arg for arg in sys.argv):
            drone_name=next((arg[2:] for arg in sys.argv if '--drone' in arg), None)

            drone=net.getNodeByName(drone_name) #stream video from a single drone

            t=threading.Thread(target=send_video, args=(drone.name, video_pathH, video_pathL, broadcast_address,
                                                        stream_src_portH, stream_src_portL, sys.argv)) #create thread sending video stream
            t.start() #start sending thread
            threads.append(t)

            drone_2_name=f'drone{int(drone_name[5])+4}'
            drone2=net.getNodeByName(drone_2_name) #stream video from spare drone after first drone goes back to base

            t2=threading.Thread(target=send_video, args=(drone2.name, video_pathH, video_pathL, broadcast_address,
                                                         stream_src_portH, stream_src_portL, sys.argv)) #create thread sending video stream
            t2.start() #start sending thread
            threads.append(t2)

        else:
            for drone in net.stations: #for each drone (Station reference)
                if drone.name=='station1':
                    continue
                t=threading.Thread(target=send_video, args=(drone.name, video_pathH, video_pathL, broadcast_address,
                                                            stream_src_portH, stream_src_portL, sys.argv)) #thread sending video stream
                t.start() #start sending thread
                threads.append(t)

    except Exception as e:
        print(e)

    if '--test' in sys.argv:
        time.sleep(5*60)
        print("Starting stress test, overloading WiFi Channel of AP1\n")
        station1=net.getNodeByName('station1')
        #station1.cmd("vlc-wrapper high_quality.mp4 --sout '#udp{dst=192.168.1.22:9999}' :no-sout-all :sout-keep")
        evil_proc=station1.popen("hping3 -c 1000000 -d 10000 -i u2000 192.168.1.22", shell=True)
        time.sleep(3*60)
        print("Stress test concluded\n")
        evil_proc.terminate()

    CLI(net) #opens Mininet CLI

    stop_event.set() #signal the threads to stop
    print(f'Stopping threads: {stop_event}\n')
    for t in threads:
        t.join() #wait for all threads to finish

    net.stop()

    print(f"\n**********Simulation has Ended**********\n")