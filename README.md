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

**sudo python3 Results.py** <br>
Options: <br>
- **[--ex1] collect telemetries for first experiment (transmit from 1 drone, receive on 4 endpoints)** <br>
- **[--ex2] collect telemetries for second experiment (transmit from 4 drones, receive on 1 endpoint + stress test on first wireless channel**
                                            
5) Run *DroneController.py* from terminal with the command:

**ryu-manager DroneController.py**

6) Run *FootballStreaming.py* from terminal with the command:

**sudo python3 FootballStreaming.py** <br>
Options: <br>
- **[--plot] use plotGraph function to visualize the network of drones and APs in a 2D space. The plot will show the movement of drones in real time** <br>
- **[--ap_tel/--dr_tel] use telemetry class to show wireless channel metrics across time measured on either the APs or the drones. Since the plot is unique, you can specify either --plot or --aps_tel/--dr_tel** <br>
- **[--rssi/--tx_bytes/--tx_packets/--tx_errors/--tx_dropped/--tx_fifo_errors/--collisions (drone metrics)/ --rx_bytes/--rx_packets/--rx_errors/--rx_missed_errors/--rx_crc_errors/--rx_dropped/--rx_over_errors/--rx_fifo_errors (AP metrics)] if either --ap_tel or --dr_tel has been specified, a metric to be observed has to be specified also. Since the plot is unique, one metric can be shown at a time** <br>
- **[--bgscan] handle AP <-> drone association through background scanning. When using background scanning, all AP will have the same SSID, allowing drones to roam. Otherwise, each AP has a dedicated SSID** <br>
- **[--manual] handle AP <-> drone association by explicitly forcing associations with setAssociation function. If --manual is not specified, auto-association of drones to APs is enabled** <br>
- **[--arp] pre-computes ARP tables statically in all endpoints and drones** <br>
- **[--set_params] set tx power, antenna gain, and supported data rate in drones and APs** <br>
- **[--endpoint1/--endpoint2/--endpoint3/--endpoint4] receive video streams only on a specific endpoints. If an endpoint is not specified, video streams will be received on all endpoints at the same time** <br>
- **[--drone1/--drone2/--drone3/--drone4] stream video only from a specific drone. If a drone is not specified, video streams will be transmitted from all drones at the same time** <br>
- **[--test] generate congestion on first WiFi Channel to test streaming adaptability**

# Debugging and Fixes:
Problems during Ryu installation:
1) Ensure for required dependencies to be installed as follows:

&emsp; &emsp; **apt install gcc python-dev libffi-dev libssl-dev libxml2-dev libxslt1-dev zlib1g-dev** <br>
&emsp; &emsp; **sudo pip3 install "gevent>=0.13" routes webob paramiko**

2) If the following error is displayed **ImportError: cannot import name 'ALREADY_HANDLED' from 'eventlet.wsgi'**, fix as follows:

&emsp; &emsp; **sudo pip3 uninstall eventlet** <br>
&emsp; &emsp; **sudo pip3 install eventlet==0.30.2**

3) If installation is successful, but **ryu-manager** is still not recognized as command, do as follows:

&emsp; &emsp; **sudo apt install python3-ryu**

<br>
Problems during Mininet-WiFi installation:
1) If the following error is displayed **fatal: detected dubious ownership in repository…**, do as follows:

&emsp; &emsp; **git config --global --add safe.directory /home/[user_name]/mininet-wifi/hostap**

2) If the following error is displayed **no such option: --break-system-packages**, do as follows:

&emsp; &emsp; **sudo nano ~/mininet-wifi/mininet/Makefile**

&emsp; &emsp; Replace the following content:

&emsp; &emsp; **python -m pip uninstall -y mininet --break-system-packages || true** <br>
&emsp; &emsp; **python -m pip install . --break-system-packages**

&emsp; &emsp; With: 

&emsp; &emsp; **python3 -m pip uninstall -y mininet || true** <br>
&emsp; &emsp; **python3 -m pip install .**

&emsp; &emsp; Then executes as follows:

&emsp; &emsp; **cd ~/mininet-wifi/mininet** <br>
&emsp; &emsp; **sudo python3 -m pip install .**

<br>

To test basic broadcast functionality, you need to enable ECHO REPLY to broadcast ping messages on Mininet hosts. To do this:
1) Open file */etc/sysctl.conf*;
2) Add line **net.ipv4.icmp_echo_ignore_broadcasts=0**;
3) Execute **sudo sysctl -p** from command line to apply configuration changes. This line is to be executed on each Mininet host (drone/endpoint) after its deployment;

# Credits
This project has been realized as assignment for the course of Smart and Programmable Networks - Module 2, A.A. 2024-2025


Co-authored by Alessandro D'Amico <alessandro99damico@gmail.com>


Master Degree in Telecommunication Engineering: Computing, Smart Sensing, and Networking


DIMES (Dipartimento di Ingegneria Informatica, Modellistica, Elettronica e Sistemistica)


UNICAL (Università della Calabria)

