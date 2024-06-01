
# Academia Innova Michelin Documentation

## Project Theme
The project theme is the recognition of colors using a Hiwonder Wonderarm robot, based on an ESP32 microcontroller, equipped with a Hiwonder WonderCam camera. The challenge was to program and demonstrate the color detection functionalities.

## ESP32 Board Setup
### Firmware Installation
The ESP32 board firmware is installed from Sparkfun. We use esptools.py from the terminal to flash the firmware onto the board.

### Adding Modules
A folder named 'drivers' was created containing the project's modules. The module used is WonderCam. We used ampy from the command line to manage the microcontroller. The WonderCam module was uploaded using the command:
```
ampy --port /dev/cu.usbserial-110 put drivers/WonderCam.py
```
where `/dev/cu.usbserial-110` is the port to which the board is connected.

## Robot Setup
### Camera Installation on Pins
The Hiwonder WonderCam was mounted on the robot and connected to specific pins (SDA, SCL) on the Hiwonder board.

### Robot Power Supply and Communication
The robot was powered using the provided adapter. Communication was done via a micro-USB cable, with the port connected to the laptop being `/dev/cu.usbserial-110`.

### Camera Setup for Color Detection
The camera was configured to detect specific colors. This involved:
- Setting the camera language
- Learning six colors (Red, Green, Blue, Yellow, Black, Purple) by associating each with an ID.

## Source Code Documentation
### WonderCam Driver
To use the WonderCam, a Python driver/module was used, obtained from the WonderArm robot documentation.

### WonderCam Starter Code
The base code for WonderCam was also taken from the documentation, specifically from the examples provided in Project 7: AI Vision Game (Color Tracking and Display).

### Source Code
```python
import time
import TM1640
from machine import Pin, I2C
from PID import PID
from WonderCam import *
from Buzzer import Buzzer
from espmax import ESPMax
from PWMServo import PWMServo
from BusServo import BusServo
from SuctionNozzle import SuctionNozzle

# 小幻熊颜色追踪并显示

# 初始化
pwm = PWMServo()
buzzer = Buzzer()
pwm.work_with_time()
bus_servo = BusServo()
arm = ESPMax(bus_servo)
nozzle = SuctionNozzle()
tm = TM1640.TM1640(clk=Pin(33), dio=Pin(32))
i2c = I2C(0, scl=Pin(16), sda=Pin(17), freq=400000)
cam = WonderCam(i2c)
cam.set_func(WONDERCAM_FUNC_COLOR_DETECT)  # 设置为颜色识别功能

if __name__ == '__main__':
  x, y, z = 0, -120, 150
  buzzer.setBuzzer(100) #设置蜂鸣器响100ms
  nozzle.set_angle(0,1000) #吸嘴角度置0
  arm.set_position((x, y, z), 2000)
  time.sleep_ms(2000)
  x_pid = PID(0.026, 0.001, 0.0008) # 设置PID参数
  z_pid = PID(0.030, 0.001, 0.0001)
  tm.update_display() # 点阵清屏

  red_buf = [0x0,0x0,0x0,0x0,0x0,0xff,0x19,0x29,0x49,0x86,0x0,0x0,0x0,0x0,0x0,0x0]

  green_buf = [0x0,0x0,0x0,0x0,0x0,0x3c,0x42,0x81,0x81,0xa1,0x62,0x0,0x0,0x0,0x0,0x0]
  blue_buf = [0x0,0x0,0x0,0x0,0x0,0xff,0x89,0x89,0x89,0x76,0x0,0x0,0x0,0x0,0x0,0x0]

while True:
    cam.update_result() # 更新小幻熊结果数据
    if cam.get_color_blob(1): # 判断是否识别id1颜色
      tm.write(red_buf) # 点阵显示‘R’
      color_data = cam.get_color_blob(1) # 获取id1颜色位置数据
    elif cam.get_color_blob(2): # 判断是否识别id2颜色
      tm.write(green_buf) # 点阵显示‘G’
      color_data = cam.get_color_blob(2) # 获取id2颜色位置数据
    elif cam.get_color_blob(3): # 判断是否识别id3颜色
      tm.write(blue_buf) # 点阵显示‘B’
      color_data = cam.get_color_blob(3) # 获取id3颜色位置数据
    else:
      tm.update_display # 点阵清屏
      color_data = None

    if color_data:
      center_x = color_data[0]
      center_y = color_data[1]

      if abs(center_x - 160) < 15: # X轴PID算法追踪
        center_x = 160
      x_pid.SetPoint = 160
      x_pid.update(center_x)
      x -= x_pid.output
      x = 100 if x > 100 else x # 机械臂X轴范围限幅
      x = -100 if x < -100 else x

      if abs(center_y - 120) < 5: # Y轴PID算法追踪
        center_y = 120
      z_pid.SetPoint = 120
      z_pid.update(center_y)
      z += z_pid.output
      z = 100 if z < 100 else z # 机械臂Z轴范围限幅
      z = 180 if z > 180 else z

      arm.set_position((x,y,z),50)  # 驱动机械臂

    time.sleep_ms(50) # 延时50ms
```

### Modifications
To achieve the necessary functionalities, the following modifications were made:
- Removed lines of code that imported/used unnecessary modules.
- Created a dictionary to associate each ID with a color.
- In each iteration of the main loop, iterated through this dictionary. If the ID matched the detected color (`cam.get_color_blob(color_id)` returns true), displayed the color name.
- If no color is identified, did not call break in the iteration, and displayed that no color was detected.
- Displayed position information (already obtained in the source code).

### Final Source Code
```python
from machine import Pin, I2C
from WonderCam import *
import time

i2c = I2C(0, scl=Pin(16), sda=Pin(17), freq=400000)
cam = WonderCam(i2c)

cam.set_func(WONDERCAM_FUNC_COLOR_DETECT)

# These are the colors corresponding to the color IDs
color_dict = {
   1: "Red",
   2: "Green",
   3: "Blue",
   4: "Yellow",
   5: "Black",
   6: "Purple"
}

if __name__ == '__main__':
   while True:
       cam.update_result() 
       color_data = None

       # Iterate through the color IDs and check if the color is detected
       for color_id, color_name in color_dict.items():

           # Returns true if the color id matches the detected color
           color_data = cam.get_color_blob(color_id)
           if color_data:
               print(color_name)
               break
       else:
           print("No color detected")

       if color_data:
           center_x = color_data[0]
           center_y = color_data[1]
           print(f"Position - x: {center_x}, y: {center_y}")

       time.sleep(1)
```

To run the code on the robot, use the command:
```
ampy --port /dev/cu.usbserial-110 run main.py
```
where:
- `/dev/cu.usbserial-110` is the port to which the board is connected.
- `main.py` is the name of the Python program uploaded.

## Results
The robot successfully detects the colors: Red, Green, Blue, Yellow, Black, and Purple under good lighting conditions. An aspect observed during testing was that colors have a range for which they can be detected. This was highlighted by shadows being detected as black and the hand being detected as purple (under the lighting conditions during testing).
