# Setup 2.4GHz Wireless USB SNES Gamepad

```
lsusb shows:
  Bus 001 Device 005: ID 0079:0126 DragonRise Inc.

Plugged in USB WiFi Gamepad adapter

sudo apt install joystick

jstest --normal /dev/input/js0

Driver version is 2.1.0.
Joystick (Controller) has 6 axes (X, Y, Z, Rz, Hat0X, Hat0Y)
and 13 buttons (BtnA, BtnB, BtnC, BtnX, BtnY, BtnZ, BtnTL, BtnTR, BtnTL2, BtnTR2, BtnSelect, BtnStart, BtnMode).
Testing ... (interrupt to exit)
Axes:  0:     0  1:     0  2:     0  3:     0  4:     0  5:     0 Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 11:off 12:off 


Pad Up:    Axis 1: -32767
Pad Dn:    Axis 1:  32767

Pad Left:  Axis 0: -32767
Pad Right: Axis 0:  32767

Select: Button 8
Start:  Button 9

X: Button 0
B: Button 2
Y: Button 3
A: Button 1

Forefinger pads:
Left:  Button 4
Right: Button 5

sudo jscal -q /dev/input/js0
jscal -u 6,0,1,2,5,16,17,13,304,305,306,307,308,309,310,311,312,313,314,315,316 /dev/input/js0


Charging: blinking red 
Fully Charged: LED off

Low Battery: blinking red
Battery OK: solid red


Instructions say do not charge more than 2 hours

Power Consumption:  12 mA at 11.13v (probably 25mA at 5v through USB)
```

=== Instruction manual for 2.4 GHz Wireless USB Controller ===  

Please read before use:  
1. Make sure the controller is fully charged prior to first use to ensure battery is at maximum capacity.  
2. Make sure to use a standard 5v 1A charger to avoid damage to the controller.  
3. Do not charge the controller for over 2 hours.  
4. The red LED will flash rapidly during normal use indicating that the controller is low on battery.  
   Please charge the controller immediately to avoid losing connection.  
5. The red LED will blink slowly when the controller is charging, it will disappear once the   
   controller is fully charged and ready for use.  
6. When the controller and the receiver cannot be connected successfully,  
   press the DOWN+SELECT+START buttons at the same time, and the red LED  
   of the controller flashes to reset.  They can be paired successfully.  


Pairing the controller:  
1. Turn on the controller by pressing the START button, the red LED will blink slowly  
2. Press the START button again, the red LED will flash rapidly indicating the controller  
   is ready to pair.  
3. Plug in the USB receiver to the PC, the red LED will stay lit  
   to indicate the controller has been paired successfully.  


LED Functions:  
1.  Press START button, LED flashes slowly: controller is "On"  
2.  With controller on, press START again, and LED flashes quickly: searching for adapter  
3.  LED on continuously:  connected/paired successfully  
4.  During charging, LED flashing every 2 seconds: charging - LED turns off when fully charged  
5.  LED flashes quickly intermittently:  battery low  


# Examples

For straight Python access, install evdev:
```
sudo pip3 install evdev

see: https://python-evdev.readthedocs.io/en/latest/usage.html

$ sudo python3 -m evdev.evtest
ID  Device               Name                                Phys                                Uniq
-----------------------------------------------------------------------------------------------------
0   /dev/input/event0    Controller                          usb-3f980000.usb-1.1.3/input0           

Select devices [0-0]: 0
Listening for events (press ctrl-c to exit) ...

=== PRESSED BUTTON A

time 1628954893.489322 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589826
time 1628954893.489322 type 1 (EV_KEY), code 305  (['BTN_B', 'BTN_EAST']), value 1
time 1628954893.489322 --------- SYN_REPORT --------
time 1628954893.657323 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589826
time 1628954893.657323 type 1 (EV_KEY), code 305  (['BTN_B', 'BTN_EAST']), value 0
time 1628954893.657323 --------- SYN_REPORT --------

=== PRESSED BUTTON B
time 1628954990.901016 --------- SYN_REPORT --------
time 1628954997.308992 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589827
time 1628954997.308992 type 1 (EV_KEY), code 306  (BTN_C), value 1
time 1628954997.308992 --------- SYN_REPORT --------
time 1628954997.468997 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589827
time 1628954997.468997 type 1 (EV_KEY), code 306  (BTN_C), value 0
time 1628954997.468997 --------- SYN_REPORT --------

=== PRESSED BUTTON X
time 1628955034.370874 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589825
time 1628955034.370874 type 1 (EV_KEY), code 304  (['BTN_A', 'BTN_GAMEPAD', 'BTN_SOUTH']), value 1
time 1628955034.370874 --------- SYN_REPORT --------
time 1628955034.528887 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589825
time 1628955034.528887 type 1 (EV_KEY), code 304  (['BTN_A', 'BTN_GAMEPAD', 'BTN_SOUTH']), value 0
time 1628955034.528887 --------- SYN_REPORT --------


=== PRESSED BUTTON Y
time 1628955067.438768 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589828
time 1628955067.438768 type 1 (EV_KEY), code 307  (['BTN_NORTH', 'BTN_X']), value 1
time 1628955067.438768 --------- SYN_REPORT --------
time 1628955067.534768 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589828
time 1628955067.534768 type 1 (EV_KEY), code 307  (['BTN_NORTH', 'BTN_X']), value 0
time 1628955067.534768 --------- SYN_REPORT --------


=== PRESSED BUTTON START
time 1628955107.970642 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589834
time 1628955107.970642 type 1 (EV_KEY), code 313  (BTN_TR2), value 1
time 1628955107.970642 --------- SYN_REPORT --------
time 1628955108.06465 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589834
time 1628955108.06465 type 1 (EV_KEY), code 313  (BTN_TR2), value 0
time 1628955108.06465 --------- SYN_REPORT --------


=== PRESSED BUTTON SELECT
time 1628955138.372549 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589833
time 1628955138.372549 type 1 (EV_KEY), code 312  (BTN_TL2), value 1
time 1628955138.372549 --------- SYN_REPORT --------
time 1628955138.488546 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589833
time 1628955138.488546 type 1 (EV_KEY), code 312  (BTN_TL2), value 0
time 1628955138.488546 --------- SYN_REPORT --------


=== PRESSED BUTTON FOREFINGER-LEFT
time 1628955164.868466 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589829
time 1628955164.868466 type 1 (EV_KEY), code 308  (['BTN_WEST', 'BTN_Y']), value 1
time 1628955164.868466 --------- SYN_REPORT --------
time 1628955165.130482 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589829
time 1628955165.130482 type 1 (EV_KEY), code 308  (['BTN_WEST', 'BTN_Y']), value 0
time 1628955165.130482 --------- SYN_REPORT --------

=== PRESSED BUTTON FOREFINGER-RIGHT
time 1628955233.102254 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589830
time 1628955233.102254 type 1 (EV_KEY), code 309  (BTN_Z), value 1
time 1628955233.102254 --------- SYN_REPORT --------
time 1628955233.31625 type 4 (EV_MSC), code 4    (MSC_SCAN), value 589830
time 1628955233.31625 type 1 (EV_KEY), code 309  (BTN_Z), value 0
time 1628955233.31625 --------- SYN_REPORT --------

=== PRESSED JOYPAD-UP
time 1628955287.718081 type 3 (EV_ABS), code 1    (ABS_Y), value 0
time 1628955287.718081 --------- SYN_REPORT --------
time 1628955287.874079 type 3 (EV_ABS), code 1    (ABS_Y), value 128
time 1628955287.874079 --------- SYN_REPORT --------


=== PRESSED JOYPAD-DN
time 1628955329.737951 type 3 (EV_ABS), code 1    (ABS_Y), value 255
time 1628955329.737951 --------- SYN_REPORT --------
time 1628955329.907947 type 3 (EV_ABS), code 1    (ABS_Y), value 128
time 1628955329.907947 --------- SYN_REPORT --------

=== PRESSED JOYPAD-RIGHT
time 1628955374.335807 type 3 (EV_ABS), code 0    (ABS_X), value 255
time 1628955374.335807 --------- SYN_REPORT --------
time 1628955374.451807 type 3 (EV_ABS), code 0    (ABS_X), value 128
time 1628955374.451807 --------- SYN_REPORT --------

=== PRESSED JOYPAD-LEFT
time 1628955400.639726 type 3 (EV_ABS), code 0    (ABS_X), value 0
time 1628955400.639726 --------- SYN_REPORT --------
time 1628955400.767729 type 3 (EV_ABS), code 0    (ABS_X), value 128
time 1628955400.767729 --------- SYN_REPORT --------



```
