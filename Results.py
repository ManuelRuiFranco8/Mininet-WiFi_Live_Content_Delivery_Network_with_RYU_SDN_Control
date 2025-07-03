#!/usr/bin/env python3

"""Usage:
   python3 Results.py [--ex1] Optional: collect telemetries for first experiment (transmit from 1 drone, receive on 4 endpoints)
                      [--ex2] Optional: collect telemetries for second experiment (transmit from 4 drones, receive on 1 endpoint + stress test on
                                        first wireless channel"""

#################################################################################################################Imports
import os
import sys
import time
import threading
import numpy as np
import matplotlib.pyplot as plt

########################################################################################################Global Variables
INTERFACES = [ "ap1-wlan1", "ap2-wlan1", "ap3-wlan1", "ap4-wlan1",
               "ap1-eth2", "ap2-eth2", "ap3-eth2", "ap4-eth2"]

INTERVAL=20
CAPACITY_MBPS_WLAN=54
CAPACITY_MBPS_ETH=1000

data_throughput={}
norm_throughput={}
timestamps={}

def get_stat_path(interface):
    if "wlan" in interface:
        return f"/sys/class/net/{interface}/statistics/rx_bytes"
    elif "eth" in interface:
        return f"/sys/class/net/{interface}/statistics/tx_bytes"
    else:
        raise ValueError(f"Not-recognized interface: {interface}")

def monitor_interface(interface, duration, interval):
    rates=[]
    norm=[]
    times=[]

    path=get_stat_path(interface)

    while not os.path.exists(path):
        print(f"{path} still does not exist\n")
        time.sleep(5)
    print(f'Proceeding Telemetry with {interface}\n')

    prev_val=0
    start_time=time.time()
    while (time.time()-start_time)<=duration:
        time.sleep(interval)

        try:
            with open(path, "r") as f:
                curr_val=int(f.read())
        except Exception as e:
            print(f"[ERROR] Failed reading for {interface}: {e}")
            break

        delta=curr_val-prev_val

        throughput=(delta*8/interval)/1_000_000 #Mbps

        if 'wlan' in interface:
            throughput_norm=(throughput/CAPACITY_MBPS_WLAN)*100 #percentage
        elif 'eth' in interface:
            throughput_norm=(throughput/CAPACITY_MBPS_ETH)*100 #percentage

        elapsed=time.time()-start_time
        rates.append(throughput)
        norm.append(throughput_norm)
        times.append(elapsed)

        prev_val=curr_val

    data_throughput[interface]=np.array(rates)
    norm_throughput[interface]=np.array(norm)
    timestamps[interface]=np.array(times)

def plot_results(group_name, interfaces, data_dict, ylabel, filename):
    linestyles = ['-', '--', '-.', ':']

    plt.figure(figsize=(10, 6))

    for idx, iface in enumerate(interfaces):
        if iface in data_dict:
            print(f"Saving telemetries for {iface}\n")
            linestyle = linestyles[idx % len(linestyles)]  # Safely wrap around if more than 4
            plt.plot(timestamps[iface], data_dict[iface], label=iface, linestyle=linestyle, linewidth=2)

    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.title(f"{ylabel} - Interfaces - {group_name}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"/home/francesco2/Documenti/PycharmProjects/Smart2/Results/{filename}.png")
    plt.close()

if __name__ == "__main__":
    threads=[]

    if '--ex1' in sys.argv:
        DURATION=600 #first experiment
    elif '--ex2' in sys.argv:
        DURATION=700 #second experiment

    for iface in INTERFACES:
        t=threading.Thread(target=monitor_interface, args=(iface, DURATION, INTERVAL))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()
        print(f"Thread Finished\n")

    wlan_ifaces=[i for i in INTERFACES if "wlan" in i]
    for wintf in wlan_ifaces:
        print(f"WLAN Interface: {wintf}\n")
        print(f"Received Throughput: {data_throughput[wintf]}\n")
        print(f"Normalized Received Throughput: {norm_throughput[wintf]}\n")
        print(f"\n")

    eth_ifaces=[i for i in INTERFACES if "eth" in i]
    for intf in eth_ifaces:
        print(f"Eth Interface: {intf}\n")
        print(f"Transmitted Throughput: {data_throughput[intf]}\n")
        print(f"Normalized Transmitted Throughput: {norm_throughput[intf]}\n")
        print(f"\n")

    plot_results("wlan", wlan_ifaces, data_throughput, "Received Throughput [Mbps]", "wlan_throughput")
    plot_results("wlan", wlan_ifaces, norm_throughput, "Normalized Received Throughput [%]", "wlan_normalized")

    plot_results("eth", eth_ifaces, data_throughput, "Transmitted Throughput [Mbps]", "eth_throughput")
    plot_results("eth", eth_ifaces, norm_throughput, "Normalized Transmitted Throughput [%]", "eth_normalized")