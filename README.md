# phoenix-firmware
[phoenix-pcb](https://github.com/Nkyoku/phoenix-pcb)のファームウェア

## Wiki
詳細は[Wiki](https://github.com/Nkyoku/phoenix-firmware/wiki)を参照

## 開発環境
- Jetson Nano Developer Kit SD Card Image 32.4.4
- ROS2 Eloquent Elusor
- Quartus Prime Lite Edition 20.1.1
- IP-NIOS (FPGAのコンパイルにNios II/fのライセンスが必要)

## ファイル
- CYPD5125-40LQXI
  - noboot.hex  
  USB PDコントローラのファームウェア
  - phoenix-cypd5125.tcl
  - firmware.tcl
  - convert_hex.py  
  J-LinkとOpenOCDでファームウェアを書き込むためのスクリプト類
  - flash.py  
  CMSIS-DAPでファームウェアを書き込むためのスクリプト
- FPGA
  - App
    - PhoenixFPGA.qpf  
    Quartus Primeプロジェクト
    - hdl  
    主要なHDLファイルとテストベンチ
    - ip  
    Platform Designerで使うIP化したHDL
    - software  
    Nios IIのファームウェア
    - output_files  
    生成ファイル
- Jetson/ros2/src
  - phoenix  
  基板上に存在するデバイスを制御するノード類を収めたパッケージ
  - phoenix_msgs  
  phoenixパッケージのノードが用いるトピックやサービスのメッセージ
  - phoenix_gui  
  phoenixパッケージのノードから受信したデータを表示したり  
  コントローラでロボットを操縦するパッケージ
- SLG46826G
  - PhoenixGP.gp6  
  電源シーケンサとして使っているGreenPAKのプロジェクト
  - PhoenixGP_without_Jetson.gp6  
  Jetsonなしで基板の電源が入るようにしたGreenPAKのプロジェクト
  - PhoenixGP_Test.gp6  
  基板のテスト用にすべての電源ICをオンするようにしたGreenPAKのプロジェクト
