from typing import List
from fastapi import FastAPI, WebSocket

app = FastAPI()

connected_clients: List[WebSocket] = []
server: WebSocket | None = None


@app.get("/status")
async def status():
    return {
        "clients": [str(client.client) for client in connected_clients],
        "server": str(server.client) if server else None,
    }


@app.websocket("/client")
async def client_handler(websocket: WebSocket):
    await websocket.accept()
    connected_clients.append(websocket)
    try:
        while True:
            data = await websocket.receive_bytes()
            if server:
                await server.send_bytes(data)
    except Exception:
        connected_clients.remove(websocket)


@app.websocket("/server")
async def server_handler(websocket: WebSocket):
    global server
    await websocket.accept()

    server = websocket

    try:
        while True:
            data = await websocket.receive_bytes()
            for client in connected_clients:
                await client.send_bytes(data)
    except Exception:
        server = None


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
