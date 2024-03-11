import asyncio
from typing import Any
import websockets
import hashlib


IP = "192.168.1.114"
# IP = "localhost:8011"

already_printed = set()


async def main(host: string):
    async with websockets.connect(
        f"ws://{host}/sp/watch", ping_timeout=3600
    ) as websocket:
        while True:
            data_: Any = await websocket.recv()
            data: bytes = data_

            tag_length = int.from_bytes(data[0:2], "big")

            tag = data[4 : 4 + tag_length]

            if tag in already_printed:
                continue

            already_printed.add(tag)
            print(tag.decode("utf-8", errors="ignore"))

if __name__ == "__main__":
    import os

    host = os.environ.get("ESP32_IP", "localhost:8011")
    asyncio.run(main(host))
