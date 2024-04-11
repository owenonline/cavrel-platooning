import socket
import json
from time import sleep

client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
client_sock.bind(("", 37020))

print("starting")
for _ in range(1000):
    data, _ = client_sock.recvfrom(1024)
    print(json.loads(data.decode()))
    sleep(0.01)