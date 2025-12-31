#!/usr/bin/env python3

import serial
import time
import threading

# Check no frame loss in high CAN bus trafic
# The maximum trafic is likely limited by the serial link speed.

def check_key():
    while True:
        if input() == 'q':
            print("quit now")
            break

check_key_thread = threading.Thread(target=check_key)
check_key_thread.start()

#device = serial.Serial("/dev/ttyACM0", timeout=1, write_timeout=1)
device = serial.Serial("COM9", timeout=1, write_timeout=1)

# Set up loopback mode with 1Mbps/5Mbps, 
# which are the maximum bitrates supported by the common CAN transceiver
device.write(b"C\r")
device.write(b"S8\r")
device.write(b"Y5\r")
device.write(b"Z0\r")
device.write(b"W2\r")
device.write(b"M00000000\r")
device.write(b"mFFFFFFFF\r")        # mFFFFFFFF -> Pass all
device.write(b"=\r")
device.write(b"z\r")                # reset cycle time stats
time.sleep(0.1)
device.read_until()
print("port_open ", device.port)
print("")

# Data frame with 64 bytes payload
data_write = b"00112233445566778899AABBCCDDEEFF"
data_write = b"b000F" + data_write + data_write + data_write + data_write + b"\r"
# Larger chunk will lead to buffer overruns in the device, lower the depth to avoid this
data_write = data_write + data_write    # Make larger chunk to improve throughput
data_write = data_write + data_write

# test duration in seconds
test_dur = 100

# max number of frames in the buffers
depth_max = 4

# initialize
depth_cur = 0
tx_len = 0
rx_len = 0
tx_cnt = 0
rx_cnt = 0

flag_tx = True

tick_1s = int(round(time.time() * 1000)) + 1000

while True:
    if flag_tx and depth_cur < depth_max:
        device.write(data_write)
        tx_len += len(data_write)
        tx_cnt += int(len(data_write) / 134)    # Number of 'b' commands
        depth_cur += int(len(data_write) / 134)

    data_read = device.read_all()
    rx_len += len(data_read)
    for byte in data_read:
        if bytes([byte]) == b"b":
            rx_cnt += 1
            depth_cur -= 1

    ms = int(round(time.time() * 1000))
    if ms > tick_1s:
        tick_1s = ms + 1000
        
        print('tx speed:', tx_len / 1000, 'kB/s\t', tx_len * 8 / 1000, 'kbits/s')
        print('rx speed:', rx_len / 1000, 'kB/s\t', rx_len * 8 / 1000, 'kbits/s')
        print('CAN trafic:', tx_cnt, "frames/s")    # ~7000 frames/s with bus load 100%
        print('CAN buffer:', depth_cur, "frames")
        print('')
        tx_len = 0
        rx_len = 0
        tx_cnt = 0
        rx_cnt = 0
        
        test_dur -= 1
        if test_dur <= 0:
            flag_tx = False

        if check_key_thread.is_alive() == False:
            break

device.read_all()
device.close()

