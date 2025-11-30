#!/usr/bin/env python3
#sudo python3 -E triggerAuth.py 
#fuser -k 5000/tcp

from scapy.all import *
import socket
import time
from datetime import datetime
from zoneinfo import ZoneInfo

# ---------- CONFIG ----------
MONITOR_IFACE = "wlp0s20f3mon"      # your monitor-mode interface
TARGET_MAC = "3e:17:e4:2a:89:c1"  # MAC you want to detect
TCP_TARGET_IP = "127.0.0.1"       # where to send TCP packet
TCP_TARGET_PORT = 5000
# -----------------------------
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_TARGET_IP, TCP_TARGET_PORT))
print("socket")

def now_ist():
    return datetime.now(ZoneInfo("Asia/Kolkata")).strftime("%Y-%m-%d %H:%M:%S.%f")

def send_tcp_trigger():
    global s
    try:
        s.sendall(b"AUTH_REQUEST_DETECTED")
        print("[+] TCP trigger sent!")

    except BrokenPipeError:
        print("[-] Broken pipe, reconnecting...")
        try:
            s.close()
        except:
            pass

        time.sleep(0.2)  # tiny delay
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((TCP_TARGET_IP, TCP_TARGET_PORT))
        print("[+] Reconnected!")
        s.sendall(b"AUTH_REQUEST_DETECTED")  # retry

def handle_packet(pkt):
    #print("in handle")
    # Ensure it is a management Authentication frame
    if pkt.haslayer(Dot11):
        if pkt.type == 0 and pkt.subtype == 0x0B:  # subtype 11 = Authentication
            src = pkt.addr2

            if src and src.lower() == TARGET_MAC.lower():
                print(f"[+] Authentication request detected from {src}")
                print(f"at {now_ist()}")
                send_tcp_trigger()

def main():
    print(f"[*] Sniffing for Authentication requests from {TARGET_MAC}...")
    sniff(iface=MONITOR_IFACE, prn=handle_packet, store=0)

if __name__ == "__main__":
    main()