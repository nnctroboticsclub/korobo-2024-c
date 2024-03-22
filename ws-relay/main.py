import asyncio
from contextlib import asynccontextmanager
import socket
from typing import List
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

connected_clients: List[WebSocket] = []
server: socket.socket | None = None
c2s_pps = 0
s2c_pps = 0


def get_self_ip():
    ip_list = [
        ip[0]
        for (fa, _, _, _, ip) in socket.getaddrinfo(socket.gethostname(), 0)
        if fa == socket.AddressFamily.AF_INET
    ]
    return ip_list


class UDPServer:
    def __init__(self, host: str, port: int):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host = host
        self.port = port

        print(f"[UDP Server] Binding to {host}:{port}")
        self.socket.bind((self.host, self.port))
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.setblocking(False)

        self.stop_ = False

    def mark_as_primary(self):
        global server

        print("[UDP Server] Marked as primary server!")
        server = self.socket

    async def recv_raw_message(self):
        data = b""
        host = ""
        port = 0
        while not self.stop_:
            try:
                data, (host, port) = self.socket.recvfrom(0x1000, 0)
                if host in get_self_ip():
                    continue
                break
            except BlockingIOError:
                await asyncio.sleep(0.01)
                continue
        return data, (host, port)

    async def worker(self):
        print("[UDP Server] Server started!")
        buffer = b""
        while not self.stop_:
            data, (_host, _port) = await self.recv_raw_message()
            buffer = buffer + data

            while len(buffer) > 4:
                length = int.from_bytes(buffer[:4], "big")
                buffer = buffer[4:]
                # print(f"[UDP Server] {host}:{port} | {length:4d} | ", end="")

                data = buffer[:length]
                buffer = buffer[length:]

                # print(f"{data.hex()}")
                for client in connected_clients:
                    try:
                        # if client.
                        await client.send_bytes(data)
                    except WebSocketDisconnect:
                        pass

            if buffer:
                print(f"[UDP Server] ==> remain: {buffer.hex()}")

        print("[UDP Server] Server stopped!")

    async def stop(self):
        self.stop_ = 1


class IPReporter:
    def __init__(self):
        self.stop_ = False

    @staticmethod
    def send_ip():
        lst = get_self_ip()
        payload = IPReporter.pack_ip_address_list(lst)

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

    def stop(self):
        self.stop_ = False

    async def worker(self):
        print("IP reporter process started!")
        print(f"Using IP List: {get_self_ip()}")
        while not self.stop_:
            await asyncio.sleep(1)
            IPReporter.send_ip()

        print("IP reporter process stopped!")


@asynccontextmanager
async def start_server(app: FastAPI):
    import os

    udp_host = os.environ.get("UDP_BRD_HOST", "0.0.0.0")

    server_receiver = UDPServer(udp_host, 8001)
    server_receiver.mark_as_primary()
    asyncio.create_task(server_receiver.worker())

    # reporter = IPReporter()
    # asyncio.create_task(reporter.worker())

    try:
        yield
    finally:
        await server_receiver.stop()
        # reporter.stop()


app = FastAPI(lifespan=start_server)


@app.get("/status")
async def status():
    return {
        "clients": [str(client.client) for client in connected_clients],
        "server": str(server.getsockname()) if server else None,
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
            if server:
                c2s_pps += 1
                print("C->S", data.hex())
                server.sendto(
                    len(data).to_bytes(4, "big") + data, ("255.255.255.255", 8001)
                )

    except WebSocketDisconnect:
        connected_clients.remove(websocket)


if __name__ == "__main__":
    import uvicorn
    import os

    uvicorn_host = os.environ.get("UVICORN_HOST", "0.0.0.0")

    uvicorn.run("main:app", host=uvicorn_host, port=8000, reload=True)
