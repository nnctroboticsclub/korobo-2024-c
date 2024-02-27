import time
from typing import List
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import socket
import multiprocessing

app = FastAPI()

connected_clients: List[WebSocket] = []
server: WebSocket | None = None
c2s_pps = 0
s2c_pps = 0


class IPReporter:
    def __init__(self):
        pass

    def send_ip(self):
        lst = IPReporter.get_ip_address_list()
        payload = IPReporter.pack_ip_address_list(lst)

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            # sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
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
        ip_list = [
            ip[0]
            for (fa, _, _, _, ip) in socket.getaddrinfo(socket.gethostname(), 0)
            if fa == socket.AddressFamily.AF_INET
        ]

        return ip_list


@app.get("/status")
async def status():
    return {
        "clients": [str(client.client) for client in connected_clients],
        "server": str(server.client) if server else None,
        "c2s_pps": c2s_pps,
        "s2c_pps": s2c_pps,
    }


@app.websocket("/client")
async def client_handler(websocket: WebSocket):
    global server, c2s_pps

    await websocket.accept()
    connected_clients.append(websocket)
    try:
        while True:
            data = await websocket.receive_bytes()
            # print("C->S", data.hex())
            if server:
                c2s_pps += 1
                await server.send_bytes(data)
    except WebSocketDisconnect:
        connected_clients.remove(websocket)


@app.websocket("/server")
async def server_handler(websocket: WebSocket):
    global server, s2c_pps
    await websocket.accept()

    server = websocket

    try:
        while True:
            data = await websocket.receive_bytes()
            # print("S->C", data.hex())
            s2c_pps += 1
            for client in connected_clients:
                await client.send_bytes(data)
    except WebSocketDisconnect as e:
        print(e)
        server = None


def reporter_process():
    print("IP reporter process ended!")
    app = IPReporter()
    while 1:
        time.sleep(1)
        if server:
            continue

        app.send_ip()
    print("IP reporter process ended!")


if __name__ == "__main__":
    proc = multiprocessing.Process(target=reporter_process)
    proc.start()

    import uvicorn

    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
