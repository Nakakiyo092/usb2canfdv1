# Benchmark for ver.2.2.0

Commit ID: ad9b1542acf56f46dbafa23251e13cb76a17e027

## Environment

USB Hub Usage: No

```powershell
PS C:\Users\kiyomaro> function prompt { "> " }
> (Get-CimInstance Win32_Processor).Name
13th Gen Intel(R) Core(TM) i7-13620H
> (Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory / 1GB
15.6787643432617
> (Get-CimInstance Win32_OperatingSystem).Caption
Microsoft Windows 11 Home
> (Get-CimInstance Win32_OperatingSystem).Version
10.0.26200
> $PSVersionTable.PSVersion.ToString()
7.6.6
> python --version
Python 3.13.14
> pip freeze
pyserial==3.5
> 
```

## CDC Speed Test

```powershell
> python ./test/cdc_speed_test.py COM9 --rx --chunk-size 16 --iteration 2 --duration 10
usb port name: COM9

serial number: N3C02
slcan version: VW1K5
detail:
    v: hardware="USB2CANFDV1", software="2.2.0", url="github.com/Nakakiyo092/usb2canfdv1"

can port status: closed

ping: [353, 239, 206, 186, 176] us

tx speed:    182.63 kB/s          1461.07 kbits/s
rx speed:    492.65 kB/s          3941.21 kbits/s
message loss:  92082  /  913168

device status: 
detail:
    z: time_ms=0x8A9F, time_us=0x09448B69, cycle_time_us_ave_max=[0x014, 0x0E4]

tx speed:    166.05 kB/s          1328.38 kbits/s
rx speed:    470.64 kB/s          3765.09 kbits/s
message loss:  45847  /  830240

device status: 
detail:
    z: time_ms=0xB343, time_us=0x09E34E60, cycle_time_us_ave_max=[0x014, 0x0E1]

> python ./test/cdc_speed_test.py COM9 --tx --chunk-size 16 --iteration 2 --duration 10
usb port name: COM9

serial number: N3C02
slcan version: VW1K5
detail:
    v: hardware="USB2CANFDV1", software="2.2.0", url="github.com/Nakakiyo092/usb2canfdv1"

can port status: closed

ping: [689, 542, 493, 471, 426] us

tx speed:    545.32 kB/s          4362.60 kbits/s
rx speed:      3.73 kB/s            29.83 kbits/s
message loss:  1946  /  39232

device status: 
detail:
    z: time_ms=0x06E1, time_us=0x0AD57479, cycle_time_us_ave_max=[0x014, 0x0DF]

tx speed:    544.66 kB/s          4357.26 kbits/s
rx speed:      3.74 kB/s            29.91 kbits/s
message loss:  1795  /  39184

device status: 
detail:
    z: time_ms=0x2F8E, time_us=0x0B7457F6, cycle_time_us_ave_max=[0x014, 0x0D4]

> 
```

## CAN Communication Test

```powershell
> python ./test/communication_test.py
.
================================================================
test_full_grid_report  (S = nominal index, Y = data index)
Legend: .=pass  E=bus_error  P=passive  O=bus-off  -=skip  ?=other

      Y0  Y1  Y2  Y3  Y4  Y5  Y6  Y7  Y8  Y9 
 S0   .   .   .   -   P   P   -   -   -   - 
 S1   .   .   E   -   P   P   -   -   -   - 
 S2   .   .   .   -   E   .   -   -   -   - 
 S3   .   .   .   -   .   .   -   -   -   - 
 S4   .   .   .   -   .   .   -   -   -   - 
 S5   .   .   .   -   .   .   -   -   -   - 
 S6   .   .   .   -   .   .   -   -   -   - 
 S7   .   .   .   -   .   .   -   -   -   - 
 S8   .   .   .   -   .   .   -   -   -   - 
 S9   -   -   -   -   -   -   -   -   -   - 

summary:  pass=39  busErr=2  passive=4  busOff=0  skip=55  other=0
================================================================
.
----------------------------------------------------------------------
Ran 2 tests in 189.281s

OK
> 
```

## CAN Stress Test

```powershell
> python ./test/can_stress_test.py --rate 2000              

=========================================================
 stress_test target devices
=========================================================
 DUT (COM9):
   slcan version: VW1K5
   serial number: N3C02
 AUX (COM8):
   slcan version: VW1K5-DEBUG
   serial number: N3C01
=========================================================
Starting: frame_type=b, S8/Y5, payload=64 bytes, rate=2000.0 fps, duration=60 s, ping_id=0x100, echo_id=0x101
Press Ctrl-C to stop early.

  t+    10s  sent=   20053  recv=   19994  loss=   59  corrupt=  0  ~ 1999.1 fps
  t+    20s  sent=   40007  recv=   39921  loss=   86  corrupt=  0  ~ 1999.5 fps
  t+    30s  sent=   60062  recv=   59934  loss=  128  corrupt=  0  ~ 1999.6 fps
  t+    40s  sent=   80011  recv=   79827  loss=  184  corrupt=  0  ~ 1999.7 fps
  t+    50s  sent=  100084  recv=   99817  loss=  267  corrupt=  0  ~ 1999.7 fps

======================================================================
 stress_test summary  (elapsed 60.0 s, target 2000.0 fps)
======================================================================
  Pings sent (DUT->AUX):           120024
  Pings observed at AUX:           119865
  Echos sent (AUX->DUT):           119865
  Echos observed at DUT:           119733
  Echos with corrupted payload:    0
  Unexpected frames at DUT:        0
  Unexpected frames at AUX:        0
  Rejected commands ([BELL]):      DUT=0  AUX=0
  End-to-end loss:                 291 (0.24 %)
  Actual achieved rate:            1999.6 fps

  DUT final F: F02
  AUX final F: F02
======================================================================
> python ./test/can_stress_test.py --rate 4000              

=========================================================
 stress_test target devices
=========================================================
 DUT (COM9):
   slcan version: VW1K5
   serial number: N3C02
 AUX (COM8):
   slcan version: VW1K5-DEBUG
   serial number: N3C01
=========================================================
Starting: frame_type=b, S8/Y5, payload=64 bytes, rate=4000.0 fps, duration=60 s, ping_id=0x100, echo_id=0x101
Press Ctrl-C to stop early.

  t+    10s  sent=   35294  recv=   21034  loss=14260  corrupt=  0  ~ 3523.5 fps
  t+    20s  sent=   70730  recv=   42075  loss=28655  corrupt=  0  ~ 3529.8 fps
  t+    30s  sent=  106054  recv=   62985  loss=43069  corrupt=  0  ~ 3533.6 fps
  t+    40s  sent=  141523  recv=   83953  loss=57570  corrupt=  0  ~ 3534.9 fps
  t+    50s  sent=  176791  recv=  104820  loss=71971  corrupt=  0  ~ 3535.4 fps

======================================================================
 stress_test summary  (elapsed 60.0 s, target 4000.0 fps)
======================================================================
  Pings sent (DUT->AUX):           212410
  Pings observed at AUX:           126098
  Echos sent (AUX->DUT):           126098
  Echos observed at DUT:           125748
  Echos with corrupted payload:    0
  Unexpected frames at DUT:        0
  Unexpected frames at AUX:        0
  Rejected commands ([BELL]):      DUT=0  AUX=0
  End-to-end loss:                 86662 (40.80 %)
  Actual achieved rate:            3538.9 fps

  DUT final F: F02
  AUX final F: F02
======================================================================
> 
```

## Long Time Test

```powershell
> python ./test/long_time_test.py COM9 --duration 3 --with-receiver
usb port name: COM9

serial number: N3C02
slcan version: VW1K5
detail:
    v: hardware="USB2CANFDV1", software="2.2.0", url="github.com/Nakakiyo092/usb2canfdv1"

can port status: open/normal (1M/5Mbps)

ping: [313, 480, 381, 343, 323, 331, 346, 312, 309, 340] us


--- Stats at 0.017 hours ---

sent frames: 1584 / 1584 (-0)
  resync-detected drops: 0

status check: 1584 samples
  no error: 1584
  buffer error: 0
  can bus error: 0

timestamp comparison (host - device): 1583 samples
  ave abs error: 23.5 us
  max abs error: 235 us
  failures: 0
    of which >32767 us: 0
    of which sentinel: 0

clock accuracy: 59 sec
  clock offset: 0.1 ms
  drift upper bound: 70.4 ppm
  drift lower bound: 0.0 ppm
  (reference, time.time() based):
    host perf_counter vs wall clock: -0.1 ppm
    device drift vs wall clock:      +1.1 ppm


... snip ...


--- Stats at 3.033 hours ---

sent frames: 266499 / 266499 (-0)
  resync-detected drops: 0

status check: 266499 samples
  no error: 266499
  buffer error: 0
  can bus error: 0

timestamp comparison (host - device): 266498 samples
  ave abs error: 24.6 us
  max abs error: 1609 us
  failures: 0
    of which >32767 us: 0
    of which sentinel: 0

clock accuracy: 10919 sec
  clock offset: 9.7 ms
  drift upper bound: 1.3 ppm
  drift lower bound: 0.5 ppm
  (reference, time.time() based):
    host perf_counter vs wall clock: -7.1 ppm
    device drift vs wall clock:      +8.0 ppm

> 
```
