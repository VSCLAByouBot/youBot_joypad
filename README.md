# youBot_joypad

如你所見，這裡現在除了這個 README 檔之外什麼都沒有

## 環境配置

+ PC 端：Ubuntu 20.04 + ROS 1 noetic
+ youBot 端：Ubuntu 16.04 + ROS 1 kinetic + youBot driver

<br>

## Wi-Fi 遠端連線
### STEP 1 | 開啟熱點

在 PC 端 wifi 設定頁面上方選單點選開啟 hotspot，並查看連線兩端的 ip

+ Hotspot 端的 IP 基本上固定為 ```10.42.0.1```
+ youBot 端的 IP 要用 ```$ ifconfig``` 查看，假設為 ```10.42.x.x```

### STEP 2 | 從電腦 ssh 遠端連進 youBot onboard PC

```python
ssh youbot@10.42.x.x
```
此終端機在後以「youBot 端 Terminal #1」稱呼

### STEP 3 |進行 ROS 傳輸

在 PC 和 youBot 兩端都輸入以下資訊，使整個 ROS 系統認定 PC 端為 master
注意每開一個終端機都需要執行（或是加入 ```~./bashrc``` 中的 if-else 選項）

```python
export ROS_MASTER_URI=http://10.42.0.1:11311    # master 端的 IP (兩端都一樣)
export ROS_IP=10.42.x.x                         # PC 或 youBot 端的 IP
```

+ PC 端 Terminal #1

  ```python
  roscore
  ```
  
+ youBot 端 Terminal #1

  ```python
  roslaunch youbot_driver_ros_interface youbot_driver.launch
  ```
+ PC 端 Terminal #2

  ```python
  rosrun <要執行的程式>
  ```

  以下為可執行檔案一覽：

  + rosrun youbot_joypad youbot_base_teleop.py
  + rosrun youbot_joypad youbot_joint_teleop.py
  + rosrun youbot_joypad youbot_teleop.py

### 連線時注意要點：

1. 無論哪一端，每個終端機都要 export ```ROS_MASTER_URI``` 和 ```ROS_IP```
2. 全部加起來只能開一個 ros master，且必須開在 ```ROS_MASTER_URI``` 指定的那一邊
3. 用 ```screen``` 可以方便在 youBot 端使用多個終端機，詳細請自行 Google
