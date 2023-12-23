# CAN 通信バスリスト

## メッセージ ID リスト

| begin |  end |     name | data       |       description       |
| ----: | ---: | -------: | :--------- | :---------------------: |
|  0x20 | 0x3F | to MDC   | XXXX       | MDC controll            |
|  0x40 | 0x40 | Control1 | TT Payload | Controller input packet |
|  0x80 | 0x80 |  ping    | FF         | ping to all device      |
|  0x81 | 0x81 |  pong    | ID 00      | pong from device        |

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