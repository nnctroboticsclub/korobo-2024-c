import asyncio
from contextlib import asynccontextmanager
import time
from typing import List
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

connected_clients: List[WebSocket] = []
server: asyncio.StreamWriter | None = None
c2s_pps = 0
s2c_pps = 0


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


@asynccontextmanager
async def start_server(app: FastAPI):
    server = AsyncTCPServer("0.0.0.0", 8001)
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
        "server": str(server.get_extra_info("peername")) if server else None,
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
                server.write(len(data).to_bytes(4, "big") + data)
    except WebSocketDisconnect:
        connected_clients.remove(websocket)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
