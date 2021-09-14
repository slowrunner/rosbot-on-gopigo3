# rosbot-on-gopigo3
Autonomous ROS2 home robot based on GoPiGo3 and RaspberryPi


<img src="https://github.com/slowrunner/rosbot-on-gopigo3/blob/main/Dave_LFQtr.jpg" width="378" height="504" />


ROSbot Specs:

- Platform: GoPiGo3 from ModularRobotics 

- Processor: Raspberry Pi 3 B+
  * 1.4 GHz Max
  * Four Cores
  * 1GB Memory
  * Onboard WiFi

- OS: Ubuntu 20.04 LTS 64-bit Server
- ROS2 Foxy: Uses 100% of one core (uptime 15min load 1.0)
 
- Control Interfaces: 
  * ssh over WiFi
  * ROS2 Foxy 
  * 2.4GHz Wireless USB "SNES" Gamepad 

- Sensors (GoPiGo3 Intrinsic)
  * Battery_Voltage (GoPiGo3 intrinsic)
  * Regulated_5v_Voltage (GoPiGo3 intrinsic)
  * Magnetic Wheel Encoders 720 cnt/rev (GoPiGo3 intrinsic)

- Sensors (Raspberry Pi Intrinsic)  
  * Processor Temperature 
  * Processor Low Voltage Throttling Active / Latched
  * Processor Temperature Throttling Active / Latched
  
- Sensors (Added):
  * YDLIDAR X4 - 360 degrees, 12cm-10m range on half degree increment, ~8Hz scanning
  * DI Distance Sensor (VL53L0X Infrared Time-Of-Flight)
    25 deg beam width, About 4% accuracy to 7.5 feet (2.3m) 
    Mounted on Tilt/Pan
  * Pi-Camera v1.3
  * DI Inertial Measurement Unit (BNO055 9DOF Fusion IMU)
    also provides ambient temperature 
  
- Actuators/Effectors (GoPiGo3 Intrinsic)
  * Wheel Motors
  * Multi-color programmable LED (x3)
  * Program controlled Red LED (x2)
  * Tri-color Battery Voltage Indicator

- Actuators/Effectors 
  * ModRobotics Servo Kit
  * USB audio+power speaker
  
- Available GoPiGo3 Ports
  * I2C: Distance Sensor
  * I2C: Unused
  * Grove Analog/Digital I/O AD1: Unused
  * Grove Analog/Digital I/O AD2: Unused 
  * SERVO1: Pan Servo

- Power Source: ModRobotics 3000mAH 11.1v Rechargeable Battery
  * 12.6v to protection circuit cutoff at 8.1-8.4v! 
  * Roughly 24wH 

- Run Time: (Using 9.75v 15minutes left "need to shutdown" limit) 
  * "Thinking" 5+ hours  (averages 414mA at 11.1v 4.6w 24wH)
  * "100% wandering" TBD hours

- Recharger:  
  * ModRobotics Li-ion Battery Charging adapter
  * 12.6v 1A output with charging/charged LED
  * About 3 hours recharge from safety shutdown

- Physical:
  * 2.5 lbs Total
  * 7" wide x 9" Long x 12" High

- Total Cost: $376

- First "Life": June 2021 
