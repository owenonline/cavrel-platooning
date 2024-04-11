import socket
import time

clientSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
clientSock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# interfaces = socket.getaddrinfo(host=socket.gethostname(), port=None, family=socket.AF_INET)
# allips = [ip[-1][0] for ip in interfaces]

# print(allips)

# while True:
#     clientSock.sendto("hello".encode(), ("255.255.255.255", 6789))
#     time.sleep(0.1)