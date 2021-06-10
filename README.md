# rosbot-on-gopigo3
Autonomous ROS2 home robot based on GoPiGo3 and RaspberryPi


![ROSbot - The ROS2 on GoPiGo3 Robot](/Graphics/ROSbot-GoPiGo3-Robot.jpg?raw=true)


ROSbot Specs:

- Platform: GoPiGo3 from ModularRobotics 

- Processor: Raspberry Pi 3 B+
  * 1.4 GHz Max
  * Four Cores
  * 1GB Memory
  * Onboard WiFi

- OS: Ubuntu 20.04 LTS 64-bit Server
 
- Control Interfaces: 
  * ssh over WiFi
  * ROS2 

- Sensors (GoPiGo3 Intrinsic)
  * Battery_Voltage (GoPiGo3 intrinsic)
  * Regulated_5v_Voltage (GoPiGo3 intrinsic)
  * Magnetic Wheel Encoders 720 cnt/rev (GoPiGo3 intrinsic)

- Sensors (Raspberry Pi Intrinsic)  
  * Processor Temperature 
  * Processor Low Voltage Throttling Active / Latched
  * Processor Temperature Throttling Active / Latched
  
- Sensors (Added):
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
  * MonkMakes 2.5W Audio Speaker (draws 9.5mA 5v at idle)
  
- Available GoPiGo3 Ports
  * I2C: Distance Sensor
  * I2C: Unused
  * Grove Analog/Digital I/O AD1: Unused
  * Grove Analog/Digital I/O AD2: Unused 
  * SERVO1: Pan Servo

- Power Source: ModRobotics 3000mAH 11.1v Rechargeable Battery
  * Cliff at ?v (0.? volts / cell)
  * Charging at ?A for about ? hours 
  * Provides around ? hours "playtime"
  
- Run Time: (Using ?v "need to shutdown" limit) 
  * "Thinking" ?
  * "100% wandering" ? hours


- Recharger:  
  * ModRobotics Li-ion Battery Charging adapter
  * 

- Physical:
  * X.X Ounces Total
  * 6" wide x 9" Long x 12" High

- First "Life": June 2021 
