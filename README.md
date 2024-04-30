# op5_gyroscope
## Reading the data from the DFRobot_WT61PC gyroscope using the Orange PI5
### Requirements:
#### 1) Serial 6-Axis Accelerometer for Arduino
   
https://www.dfrobot.com/product-2200.html

<img src = "https://github.com/DFRobot/DFRobot_WT61PC/blob/master/resources/images/WT61PC.png">

#### 2) Orange PI 5 (plus)

<img src = "http://www.orangepi.org/img/pi5-plus/pi5-plus-5.png">

The repository uses the sources for Arduino: https://github.com/DFRobot/DFRobot_WT61PC

We just re-implemented it for the Orange PI.

Steps to start:

1) Install the Wiring OP:
```
cd wiringOP
sudo ./build clean
sudo ./build
```
2) Connect the device:

```
sudo orangepi-config
```
Select **System** -> **Hardware**, enable config **uart1-m1**

Connect the gyro to the board:

**Gyro TX->27, Gyro RX->28, VCC->2, GND->9**
```
ls /dev/ttyS*
```
Should be printed:

```
/dev/ttyS1  /dev/ttyS9
```
Now, clone the repo, run and enjoy!

The output looks like:

```
Acc:    -0.01     0.05    10.05

Gyro:     0.00     0.00     0.00

Angle:     0.13     0.05   165.89
```
