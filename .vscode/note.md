# めもとか

## Robo++ の依存関係グラフ

```mermaid
graph RL;
  ikakoMDCMotor --> ikakoMD;
  ikakoMDCEncoder --> ikakoMD;
  Motor --> ikakoMDCMotor;
  Node --> Motor;
  Node --> ikakoMDCEncoder;

  Node --> swerveMotor;
  AngledMotor --> swerveMotor;
  Joystick2Angle --> swerveMotor;

  PID --> AngledMotor;
  Node --> AngledMotor;

  Node --> Joystick2Angle;
  JoyStick2D --> Joystick2Angle;
  AngleStick2D --> Joystick2Angle;

  Node --> Swerve;
  Joystick2D --> Swerve;
  swerveMotor --> Swerve;
  PID --> Swerve;
  AngleNormalizer --> Swerve;
  Muxer --> Swerve;

  Node --> AngleNormalizer;
  Node --> Muxer;

  Node --> AngleClamper;

  PID --> IncAngledMotor;
  Node --> IncAngledMotor;
  AngleNormalizer --> IncAngledMotor;
  AngleClamper --> IncAngledMotor;

  Node --> PID;

  Motor --> BLDC;
  Node --> Gyro;
  Gyro --> BNO055;
```

## タスク一覧

- [ ] (はやくやつ) BNO055 が基板上でつながらない
- [ ] (はやくやる) ikakoMDC がつながらない --> 頑張る
- [ ] (はやめにやる) 接続が不安定 --> ESP32 をクライアントにする？
- [ ] (いつかやる) エンコーダーのノードを作成 --> 頑張る
- [ ] (いつかやる) Pipe を実装 <== Link#Node 使いづらい