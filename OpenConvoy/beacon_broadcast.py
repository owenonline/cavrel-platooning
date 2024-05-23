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
from crossplatform import ROSArgs

EARTH_RADIUS = 6371e3
DUE_EAST = 90

class Beacon:
    def __init__(self, args:ROSArgs):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        ttl = struct.pack('b', 1)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)