import socket
import struct
from time import sleep

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
ttl = struct.pack('b', 1)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

while True:
    sent = sock.sendto("Hello".encode(), ('224.0.0.1', 5004))
    sleep(0.1)