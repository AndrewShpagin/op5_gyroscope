# op5_gyroscope
## Reading the data from the DFRobot_WT61PC gyroscope using the Orange PI5
<img src = "https://github.com/DFRobot/DFRobot_WT61PC/blob/master/resources/images/WT61PC.png">
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

