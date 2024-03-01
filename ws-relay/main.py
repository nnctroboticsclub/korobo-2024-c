import asyncio
from contextlib import asynccontextmanager
import socket
import threading
from typing import List
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

connected_clients: List[WebSocket] = []
server: socket.socket | None = None
c2s_pps = 0
s2c_pps = 0

"""
class AsyncTCPServer:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.server = None

    async def start(self):
        self.server = await asyncio.start_server(
            self.handle_client, self.host, self.port
        )
        print(f"Server started at {self.host}:{self.port}")

    async def handle_client(self, reader, writer: asyncio.StreamWriter):
        global server, s2c_pps
        print(f"New connection from {writer.get_extra_info('peername')}")
        server = writer
        try:
            while True:
                data = await reader.read(4)
                if not data:
                    break
                length = int.from_bytes(data, "big")

                data = await reader.read(length)
                if not data:
                    break

                s2c_pps += 1
                for client in connected_clients:
                    await client.send_bytes(data)
        finally:
            print("Connection closed")
            server = None

    async def stop(self):
        if self.server:
            self.server.close()
            await self.server.wait_closed()
            print("Server stopped")
"""


class UDPServer:
    def __init__(self, host: str, port: int):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host = host
        self.port = port
        self.thread: threading.Thread

        self.stop_ = False

    async def start(self):
        global server

        self.socket.bind((self.host, self.port))
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.setblocking(False)

        server = self.socket
        self.thread = threading.Thread(target=lambda: self.worker())
        self.thread.start()

    def worker(self):
        loop = asyncio.new_event_loop()
        loop.run_until_complete(self.loop())
        loop.close()

    async def loop(self):
        buffer = b""
        while not self.stop_:
            print("Waiting Packet...")
            while not self.stop_:
                try:
                    data, addr = self.socket.recvfrom(0x1000, 0)
                    break
                except BlockingIOError:
                    continue
            buffer = buffer + data

            while len(buffer) > 4:
                length = int.from_bytes(buffer[:4], "big")
                buffer = buffer[4:]
                print(f"- Length: {length}, From: {addr}")

                data = buffer[:length]
                buffer = buffer[length:]

                print(f"- data: {data.hex()}, From: {addr}")
                for client in connected_clients:
                    await client.send_bytes(data)

            print(f"==> remain: {buffer.hex()}")

    async def stop(self):
        self.stop_ = 1
        await asyncio.sleep(1)


@asynccontextmanager
async def start_server(app: FastAPI):
    print("lifespan!")
    server = UDPServer("0.0.0.0", 8001)
    await server.start()

    try:
        yield
    finally:
        await server.stop()


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

    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
