import time
from typing import List
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

app = FastAPI()

connected_clients: List[WebSocket] = []
server: WebSocket | None = None
c2s_pps = 0
s2c_pps = 0


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
            print(
                f"{time.strftime('%H:%M:%S', time.localtime())} C->S: {len(data)}bytes"
            )
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
            print(
                f"{time.strftime('%H:%M:%S', time.localtime())} S->C: {len(data)}bytes"
            )
            s2c_pps += 1
            for client in connected_clients:
                await client.send_bytes(data)
    except WebSocketDisconnect as e:
        print(e)
        server = None


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
