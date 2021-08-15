#!/usr/bin/env python3

# FILE:  test_joy.py

# PURPOSE:  Raw Python access to "joystick at /dev/input/js0"

# USAGE:  sudo ./test_joy.py

import evdev
import time

print("Enumerating Input Devices")
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
for device in devices:
    print(device.path, device.name, device.phys)
    print(device.capabilities(verbose=True))
    gamepad_device_path = device.path
print("Enumeration Complete")

if len(devices) == 0:
  print("GamePad Not Found")
  exit(0)

print("Setting Up GamePad on {}".format(gamepad_device_path))
gamepad = evdev.InputDevice(gamepad_device_path)
print(gamepad)

print("Check Gamepad Power LED is ON")
time.sleep(1)

print("Watching for gamepad events (CTRL-C to STOP WATCHING)")
try:
  for event in gamepad.read_loop():
    if event.type == evdev.ecodes.EV_KEY:
      print(evdev.categorize(event))
except KeyboardInterrupt:
  print("\nCtrl-C Detected - Exiting event read_loop()")

print("Polling for Active Keys (CTRL-C to STOP POLLING)")
while True:
  try:
    print("Active keys:{}:".format(gamepad.active_keys()))
    time.sleep(0.1)
  except KeyboardInterrupt:
    print("\nCtrl-C Detected, exiting Active Key Polling")
    break

print("Polling for JoyPad Keys (CTRL-C to STOP POLLING)")
while True:
  try:
    print("Left-Right:{}:".format(gamepad.absinfo(0).value))
    print("Up-Down:{}:".format(gamepad.absinfo(1).value))
    time.sleep(0.1)
  except KeyboardInterrupt:
    print("\nCtrl-C Detected, exiting JoyPad Key Polling")
    break


# Stuck in some weird event loop
print("\nPress Ctrl-C again to EXIT")
# No Effect
gamepad.close()

exit(0)
