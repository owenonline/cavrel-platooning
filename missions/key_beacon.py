import random
import socket
import struct
import pyproj
import json
import math
from time import time, sleep
import argparse
import threading
import numpy as np
from pynput import keyboard

# Set up for broadcast
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
ttl = struct.pack('b', 1)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

to_send = {"keys": []}
key_pressed = set()
lock = threading.Lock()

def on_press(key):
    global to_send, key_pressed
    with lock:
        try:
            if key.char in ['w', 'a', 's', 'd', 'p']:
                key_pressed.add(key.char)
        except AttributeError:
            pass

def on_release(key):
    global to_send, key_pressed
    with lock:
        try:
            if key.char in ['w', 'a', 's', 'd', 'p']:
                key_pressed.discard(key.char)
        except AttributeError:
            pass

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

while True:
    with lock:
        to_send["keys"] = list(key_pressed)

    msg = json.dumps(to_send)
    print(msg)
    msg = msg.encode()
    sock.sendto(msg, ('224.0.0.1', 5004))

    # Wait for next broadcast interval
    sleep(0.1)
