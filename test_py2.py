import socket
import json
from time import sleep

client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
client_sock.bind(("", 37020))

hostname = socket.gethostname()
interfaces = socket.getaddrinfo(hostname, None, socket.AF_INET)
allips = [ip[-1][0] for ip in interfaces]

msg = "TEST".encode()
for _ in range(1000):
    print("broadcasting TEST")
    for ip in allips:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.bind((ip,0))
        sock.sendto(msg, ("255.255.255.255", 37020))
        sock.close()
    sleep(0.1)

# for _ in range(1000):
#     data, _ = client_sock.recvfrom(1024)
#     data_json = json.loads(data.decode())
#     print(data_json)
#     sleep(0.01)
