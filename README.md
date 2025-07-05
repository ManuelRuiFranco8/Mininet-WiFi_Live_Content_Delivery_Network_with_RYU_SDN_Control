# VM Environment Set-Up:
- Ubuntu 20.04
- Python 3.8.10
- Ryu 4.34
- Mininet-WiFi 2.6
- Open vSwitch 3.5.0
- Pandas 2.0.3 / Numpy 1.24.4 / Matplotlib 3.1.2 / Scapy 2.6.1
- VLC Media Player 3.0.9.2
- Hping3 3.0.0


# Codes Explained:

*FootballStreaming.py* deploys an end-to-end content delivery network using Mininet-WiFi library. Use case: live streaming of a football match.
Network consists of 8 drones (Mininet-emulated Hosts) flying and hovering over a football field, transmitting live video streams to fixed endpoint devices (Mininet-emulated Hosts).
Drone devices are connected to APs (Open vSwitch kernel datapaths) via emulated Wifi 802.11a channels (Wmediumd). An Ethernet backbone (Linux Traffic Control) connects APs among themselves and with client endpoints. Network IPv4 addressing is static, in the range 192.168.1.0/24.
Drones mobility is emulated by Mininet-WiFi in replay mode from a series of .dat files (which are written upon running the code).
Each drone transmits two streams at the same time (simulating quality-scalable streaming), which are received on all endpoints (simulating a multi-camera video streaming).
This code opens VLC processes to transmit and receive .mp4 files as live video streams (RTSP protocol) on Mininet-emulated Hosts. It also run threads allowing each drone to notify its current position and current WiFi channel occupation to the controller.
It opens a TCP socket on localhost:8080 to receive drone mobility commands from the controller. File *CommandsLog.txt* records all commands received from the controller.

*SendPosition.py* is executed on Mininet-emulated drone devices to transmit in-band notifications to controller using Scapy library. Notification conteins drones current coordinates and current sensed occupation of the WiFi channel. File *drone8_notification.txt* cointains a sample notification packet.

*DroneController.py* runs a Ryu application on localhost:6653. Ryu controller proactively installs flow rules, group rules, and meter rules on OVS access points to enable video streaming on the emulated network.
A double optimization of the video streaming is implemented:
1. Dynamically adapt stream transmission from drones according to current WiFi channel occupation (i.e. drop transmission of high-quality stream if channel is congested);
2. Static service differentiation between endpoints (i.e., certain endpoints will receive low-quality streams, while other will receive high-quality streams);
A thread also keeps track in real time of drones energy consumption according to received notifications and theoretical models, sending mobility commands to drones on a TCP socket opened on localhost:8080.
Controller also proactively installs flow rules and group rules for IPv4 broadcasting and installs reactively flow rules for ARP traffic and IPv4 unicast traffic.
File *ControllerLog.txt* records all rules installed by the controller on APs. File *MobilityLog.txt* records all position notifications received from drones and all transmitted mobility commands sent to drones. File *BandwidthLog.txt* records all channel occupation feedbacks received from drones.

*Results.py* collects real-time statistics (transmitted/received bitrate) from virtual L2 interfaces of APs into numpy series, plotting them with Matplotlib and saving them to .png.


# How to run a simulation:
1) Set-up identical VM environment;
2) Download all the codes in you VM environment;
3) Set in the codes all correct paths for .dat, .mp4, and .txt files in your VM environment;
4) Run *Results.py* from terminal with the command:

**sudo python3 Results.py [--ex1] Optional: collect telemetries for first experiment (transmit from 1 drone, receive on 4 endpoints)** <br>
&emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; **[--ex2] Optional: collect telemetries for second experiment (transmit from 4 drones, receive on 1 endpoint + &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; stress test on first wireless channel**
                                            
5) Run *DroneController.py* from terminal with the command:

**ryu-manager DroneController.py**

6) Run *FootballStreaming.py* from terminal with the command:

**sudo python3 FootballStreaming.py [--plot] Optional: use plotGraph function to visualize the network of drones and APs in a 2D space. The &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; plot will show the movement of drones in real time** <br>
&emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; **[--ap_tel/--dr_tel] Optional: use telemetry class to show wireless channel metrics across time measured &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp;  on either the APs or the drones. Since the plot is unique, you can specify either --plot or &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; --aps_tel/--dr_tel** <br>
&emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; **[--rssi/--tx_bytes/--tx_packets/--tx_errors/--tx_dropped/--tx_fifo_errors/--collisions (drone metrics)/ &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; --rx_bytes/--rx_packets/--rx_errors/--rx_missed_errors/--rx_crc_errors/--rx_dropped/--rx_over_errors/ &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; --rx_fifo_errors (AP metrics)] Optional: if either --ap_tel or --dr_tel has been specified, a metric to be &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; observed has to be specified also. Since the plot is unique, a single metric can be shown at a time** <br>
&emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; **[--bgscan] Optional: handle AP <-> drone association through background scanning. When using &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; background scanning, all AP will have the same SSID, allowing drones to roam. Otherwise, each AP has a dedicated SSID** <br>
&emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; **[--manual] Optional: handle AP <-> drone association by explicitly forcing associations with &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; setAssociation function. If --manual is not specified, auto-association of drones to APs is enabled** <br>
&emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; **[--arp] Optional: pre-computes ARP tables statically in all endpoints and drones** <br>
&emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; **[--set_params] Optional: set tx power, antenna gain, and supported data rate in drones and APs** <br>
&emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; **[--endpoint1/--endpoint2/--endpoint3/--endpoint4] Optional: receive video streams only on a specific &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; endpoints. If an endpoint is not specified, video streams will be received on all endpoints at the same time** <br>
&emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; **[--drone1/--drone2/--drone3/--drone4] Optional: stream video only from a specific drone. If a drone is &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; not specified, video streams will be transmitted from all drones at the same time** <br>
&emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; &emsp; **[--test] Optional: generate congestion on first WiFi Channel to test streaming adaptability**


# Debugging:
To test basic broadcast functionality, you need to enable ECHO REPLY to broadcast ping messages on Mininet hosts. To do this:
1) Open file */etc/sysctl.conf*;
2) Add line **net.ipv4.icmp_echo_ignore_broadcasts=0**;
3) Execute **sudo sysctl -p** from command line to apply configuration changes. This line is to be executed on each Mininet host (drone/endpoint) after its deployment;

# Credits
This project has been realized as assignment for the course of Smart and Programmable Networks - Module 2, A.A. 2023-2024


Co-authored by Alessandro D'Amico <alessandro99damico@gmail.com>


Master Degree in Telecommunication Engineering: Computing, Smart Sensing, and Networking


DIMES (Dipartimento di Ingegneria Informatica, Modellistica, Elettronica e Sistemistica)


UNICAL (Università della Calabria)

