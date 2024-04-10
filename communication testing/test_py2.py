import socket
import json
from time import sleep

client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
client_sock.bind(("", 6789))

msg = "TEST".encode()
for _ in range(1000):
    print("broadcasting TEST")
    client_sock.sendto(msg, ("192.168.0.255", 6789))
    sleep(0.1)
