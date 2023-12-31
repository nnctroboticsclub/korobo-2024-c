# CAN 通信バスリスト

## デバイス ID リスト

| ID        | name  | description          |
| --------- | ----- | -------------------- |
| 0         | STM32 | Main STM32 board     |
| 1         | STM32 | Encoder STM32 board  |
| 2         | ESP32 | ESP32 for connection |

## メッセージ ID リスト

| begin |  end |         name | data       |       description       |
| ----: | ---: | -----------: | :--------- | :---------------------: |
|  0x20 | 0x3F |       to MDC | XXXX       | MDC controll            |
|  0x40 | 0x40 |     Control1 | TT Payload | Controller input packet |
|  0x60 | 0x60 |    Query Val | ID ID      | Query value             |
|  0x61 | 0x61 | Value notify | TT Payload | Value Notify            |
|  0x70 | 0x7x |    CAN Error | XX XX      | TX, RX Errors           |
|  0x80 | 0x80 |         ping |            | ping to all device      |
|  0x81 | 0x8F |         pong |            | pong from device        |

## データ型

### コントローラー入力パケット

|  01  |  2  |  34567  | Payload | description     |
| ---- | --- | ------- | ------- | --------------- |
|  00  |  0  |   ID    | 0 byte  | button pressed  |
|  00  |  1  |   ID    | 0 byte  | button released |
|  01  |  0  |   ID    | 1 byte  | Joystick X      |
|  01  |  1  |   ID    | 1 byte  | Joystick Y      |
|  10  |  0  |   ID    | 0 byte  | Do Action (ID)  |
|  11  |  0  |   ID    | 0 byte  | Stop driving    |
|  11  |  1  |   ID    | 0 byte  | Start driving   |