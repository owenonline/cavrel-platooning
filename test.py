import socket
import json
from time import sleep

client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
# client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
# client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
client_sock.bind(("", 37020))

# interfaces = socket.getaddrinfo(host=socket.gethostname(), port=None, family=socket.AF_INET)
# allips = [ip[-1][0] for ip in interfaces]

# msg = "TEST".encode()
# for _ in range(1000):
#     for ip in allips:
#         sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
#         sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
#         sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
#         sock.bind((ip,0))
#         sock.sendto(msg, ("255.255.255.255", 37020))
#         sock.close()
#     sleep(0.01)
print("starting")
for _ in range(1000):
    print("listening")
    data, _ = client_sock.recvfrom(1024)
    print("listening")
    print(data.decode())
    sleep(0.01)