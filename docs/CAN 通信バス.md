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

| 012 |  34567  | Payload | description     |
| --- | ------- | ------- | --------------- |
| 000 |   ID    | 0 byte  | button pressed  |
| 001 |   ID    | 0 byte  | button released |
| 010 |   ID    | 1 byte  | Joystick X      |
| 011 |   ID    | 1 byte  | Joystick Y      |
| 100 |   ID    | 0 byte  | Do Action (ID)  |
| 110 |   ID    | 0 byte  | Stop driving    |
| 111 |   ID    | 0 byte  | Start driving   |

## 交流ロボコン 2023 長岡 C 用コントローラー ID リスト

| type | ID  | name           | description |
| ---- | --- | -------------- | ----------- |
|  00x | 0   | Launch         | 射出         |
|  000 | 1   | Steer Rot PID  | ステア角 PID  |
|  001 | 1   | Steer Rot Raw  | ステア角速度  |
|  010 | 0   | Steer Move     | ステア移動    |
|  010 | 0   | Steer Rotation | ステア角度    |