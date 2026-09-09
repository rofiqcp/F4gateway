#!/usr/bin/env python3
"""Send one set of representative ADV telemetry values to the STM32 HMI."""
import sys
import time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
lines = [
    "SYS:READY",
    "MODE:MANUAL",
    "STATE:STOPPED",
    "SPD:0.0",
    "HEAD:32.0",
    "GPS:1",
    "FIX:3",
    "LAT:-7.050123",
    "LON:110.440235",
    "SAT:17",
    "HDOP:0.82",
    "IMU:1",
    "CAM:1",
    "PER:1",
    "OBJ:PERSON",
    "DIST:3.24",
    "CONF:92",
    "DRV:1",
    "OBS:0",
    "STEER_TARGET:15.0",
    "STEER_ACTUAL:14.8",
    "STEER_ERR:0.2",
    "RPM:328",
    "ESC:1",
    "ENC:1",
    "MANUAL_SPEED:20",
]

with serial.Serial(port, 1000000, timeout=0.3) as ser:
    time.sleep(0.4)
    for line in lines:
        ser.write((line + "\n").encode())
        time.sleep(0.03)
    ser.write(b"GET:STATE\n")
    time.sleep(0.2)
    while ser.in_waiting:
        print(ser.readline().decode(errors="replace").rstrip())
