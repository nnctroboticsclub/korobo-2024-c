import asyncio
from typing import Any
import websockets
import hashlib

# IP = "192.168.1.219"
IP = "localhost:8011"


async def main():
    async with websockets.connect(
        f"ws://{IP}/sp/watch", ping_timeout=3600
    ) as websocket:
        prev_received_message_hash = None
        while True:
            data_: Any = await websocket.recv()
            data: bytes = data_

            data_hash = hashlib.sha1(data).digest()
            if prev_received_message_hash:
                if data_hash == prev_received_message_hash:
                    continue

            prev_received_message_hash = data_hash

            tag_length = int.from_bytes(data[0:2], "big")
            buf_length = int.from_bytes(data[2:4], "big")

            tag = data[4 : 4 + tag_length]
            buf = data[4 + tag_length : 4 + tag_length + buf_length]

            print(buf.decode("utf-8", errors="ignore"), end="")


if __name__ == "__main__":
    asyncio.run(main())
