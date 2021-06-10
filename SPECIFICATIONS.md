Feature / Bot	GoPiGo3 “ROSbot”

| Feature | Detail |
| --- | --- |
| Motors | Quantity 2 |
| | 6v Gear Reduction (1:120) motors |
| | 4500 mg-cm at 6v |  
| | drive() batt. load: ~330mA at 9.5v | 
| Speed | 2.6 in/sec at 120 DPS |
| | 3.25 in/sec 82.6mm at 150 DPS |
| | 6.4 in/sec at 300 DPS (default)|
| | 43 in/sec (1.1m/s) max acceleration |
| | 7.7  in/sec (0.2m/s) at 360 DPS (max for  straight travel) |
| Supply Circuit |Rohm DC/DC Buck BD9D320EFJ |
| | 4.5-18v input |
| | 5v 3A |
| Tires	| 2.6” 66.5mm | 
| Robot Dimensions | 9 L x 5 W x 7 H |
| Chassis Shape	| Three Layer Rectangle|
| Weight | X.x lbs |
| Min Turning Radius | 14cm 5.5 inch |
| | "Axle" center to rear corners |

19cm 7.5 inch orbit radius
Wheel center to opp. rear corner 

Using WDia,WBase	64.0 (2.5")  114.05 mm (4.5")
	
“Grove” Analog / Digital / PWM :     (A2D 0-5v 12bit 1.2mV resolution)	Grove Connector: 
1-A: pin 1 (default)
1-B: pin 2
2-A: pin 1 (default)
2-B: pin 2
pin 3: VCC 5v
pin 4: GND 

Serial Port	1 level converted port: unused

Servo Ports	2 ports: 1) pan  2) unused

Motor Driver	Dual TB6612FNG Dual 1.2A 15v max

I2C 
(I2C ports are parallel wired -interchangeable)	2x “Grove” HW I2C ports: 
    (level converted) 
    “1”) DI Distance Sensor  
    “2”) unused

Roller	(Original metal ball replaced with:) 
3/4” Pololu plastic ball castor wheel

Power Switch	Push/Push

Pan	TowerPro SG90 Micro Servo

Sensors:	

   Battery V	12bit intrinsic float, 16:1 div, 
8.6mV precision, 70mv variation

   Distance TOF	VL53L0X  I2C $20 1-78” +/-12.5deg beam (~36” at max detect range)
~20ms continuous , ~120ms single reading 1%-5% variation, 6 reading average 1%-3% variation
I2C 0x2A

   Pi Camera v1.3
   - 5MP 1080p30 2592 x 1944 pixels
     35mm focal length equiv.
     focus 1 m to inf. 
     H 53 deg x V 41 deg
     f/2.9
     (Use 1296 x 976 and 2x2 binning for better low light SNR) , Flex Cable 18”/457mm

   Encoders	720 counts / rev   (6 count/motor_rev x 120:1 gear reduction)
(APIs in degrees, 360 / wheel rev)

    Microphone	None

Indicators	

   Battery On/Level	Red/Yellow/Purple Voltage Indicator

   Other	3x Multi-Color WS2812B LED
	2x Red LED

Processor	Raspberry Pi 3 Model B+
* 1.4GHz Four-core Cortex-A53
* 1 GB RAM
* 32GB SD card / ~8GB used 
* 600ma idle - 1.34A full load

ADC 	ATMEL SAMC20J
	12-bit with 2.048v ± 0.041v Vref 

Power Source	ModRobotics 3000mAH 11.1 Rechargeable LI-Ion Battery

Configured Idle Battery Current Draw              225ma 11v - 308mA 8v 
ave: 258mA 9.6v  2.5w         
equiv. 500mA at 5v                          
Active Current Draw ( Motors, Camera, Sensors, Servo )	800 mA - 2A
Run Time	4-5 h (~30Wh), 

Extended ROSbot Cost	$304 June 2021
