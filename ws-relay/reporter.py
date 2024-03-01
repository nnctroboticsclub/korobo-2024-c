import asyncio
import socket
from typing import List


class IPReporter:
    def __init__(self):
        pass

    def send_ip(self):
        lst = IPReporter.get_ip_address_list()
        payload = IPReporter.pack_ip_address_list(lst)
        print("Sending ips:", lst)

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
            sock.sendto(payload, ("255.255.255.255", 38000))

    @staticmethod
    def pack_ip_address_list(lst: List[str]):
        payload = b""
        payload += len(lst).to_bytes(4, "big")
        for ip in lst:
            payload += len(ip).to_bytes(4, "big")
            payload += ip.encode()

        return payload

    @staticmethod
    def get_ip_address_list():
        # return ["192.168.22.222"]
        ip_list = [
            ip[0]
            for (fa, _, _, _, ip) in socket.getaddrinfo(socket.gethostname(), 0)
            if fa == socket.AddressFamily.AF_INET
        ]

        return ip_list


async def reporter():
    print("IP reporter process started!")
    app = IPReporter()
    while 1:
        await asyncio.sleep(1)
        app.send_ip()
    print("IP reporter process ended!")


if __name__ == "__main__":
    asyncio.run(reporter())
