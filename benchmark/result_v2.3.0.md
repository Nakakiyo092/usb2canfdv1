# Benchmark for ver.2.3.0

Commit ID: 77538720e5631c8a0b581cc814880fc746bd9a21

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
slcan version: VW1K6
detail:
    v: hardware="USB2CANFDV1", software="2.3.0", url="github.com/Nakakiyo092/usb2canfdv1"

can port status: closed

ping: [340, 244, 223, 177, 171] us

tx speed:    193.63 kB/s          1549.00 kbits/s
rx speed:    516.67 kB/s          4133.35 kbits/s
message loss:  107014  /  968128

device status: 
detail:
    z: time_ms=0x5167, time_us=0x16B325CF, cycle_time_us_ave_max=[0x014, 0x0E1]

tx speed:    195.61 kB/s          1564.88 kbits/s
rx speed:    504.75 kB/s          4038.00 kbits/s
message loss:  136799  /  978048

device status: 
detail:
    z: time_ms=0x7A0C, time_us=0x1751EBD6, cycle_time_us_ave_max=[0x014, 0x0DC]

> python ./test/cdc_speed_test.py COM9 --tx --chunk-size 16 --iteration 2 --duration 10
usb port name: COM9

serial number: N3C02
slcan version: VW1K6
detail:
    v: hardware="USB2CANFDV1", software="2.3.0", url="github.com/Nakakiyo092/usb2canfdv1"

can port status: closed

ping: [353, 350, 362, 374, 359] us

tx speed:    560.23 kB/s          4481.80 kbits/s
rx speed:      3.69 kB/s            29.53 kbits/s
message loss:  3389  /  40304

device status: 
detail:
    z: time_ms=0xB820, time_us=0x18446772, cycle_time_us_ave_max=[0x013, 0x0D4]

tx speed:    565.12 kB/s          4520.95 kbits/s
rx speed:      3.66 kB/s            29.29 kbits/s
message loss:  4040  /  40656

device status: 
detail:
    z: time_ms=0xE0C5, time_us=0x18E32D46, cycle_time_us_ave_max=[0x014, 0x0D4]

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
 S0   .   E   P   -   P   P   -   -   -   - 
 S1   .   .   .   -   P   P   -   -   -   - 
 S2   .   .   .   -   .   .   -   -   -   - 
 S3   .   .   .   -   .   .   -   -   -   - 
 S4   .   .   .   -   .   .   -   -   -   - 
 S5   .   .   .   -   .   .   -   -   -   - 
 S6   .   .   .   -   .   .   -   -   -   - 
 S7   .   .   .   -   .   .   -   -   -   - 
 S8   .   .   .   -   .   .   -   -   -   - 
 S9   -   -   -   -   -   -   -   -   -   - 

summary:  pass=39  busErr=1  passive=5  busOff=0  skip=55  other=0
================================================================
.
================================================================
test_sp_grid_report  (nominal S8, rows = data SP, columns = data bit rate)
CAN clock: DUT 80 MHz, AUX 80 MHz
Legend: .=pass  E=bus_error  P=passive  O=bus-off  -=not representable  ?=other

         1M   2M   4M   5M   8M  10M  16M  20M
  20%    .    .    .    -    O    -    -    - 
  25%    .    .    .    .    -    O    -    - 
  30%    .    .    .    -    .    -    -    - 
  35%    .    .    .    -    -    -    -    - 
  40%    .    .    .    -    .    -    O    - 
  45%    .    .    .    -    -    -    -    - 
  50%    .    .    .    .    .    .    -    O 
  55%    .    .    .    -    -    -    -    - 
  60%    .    .    .    -    .    -    .    - 
  65%    .    .    .    -    -    -    -    - 
  70%    .    .    .    -    .    -    -    - 
  75%    .    .    .    .    -    .    -    . 
  80%    .    .    .    -    .    -    .    - 
  85%    .    .    .    -    -    -    -    - 
  90%    .    .    .    -    .    -    -    - 
  95%    .    .    .    -    -    -    -    - 

summary:  pass=63  busErr=0  passive=0  busOff=4  skip=61  other=0
================================================================
.
----------------------------------------------------------------------
Ran 3 tests in 360.825s

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
   slcan version: VW1K6
   serial number: N3C02
 AUX (COM8):
   slcan version: VW1K6-DEBUG
   serial number: N3C01
=========================================================
Starting: frame_type=b, S8/Y5, payload=64 bytes, rate=2000.0 fps, duration=60 s, ping_id=0x100, echo_id=0x101
Press Ctrl-C to stop early.

  t+    10s  sent=   20057  recv=   20051  loss=    6  corrupt=  0  ~ 2000.5 fps
  t+    20s  sent=   40006  recv=   39993  loss=   13  corrupt=  0  ~ 2000.3 fps
  t+    30s  sent=   60090  recv=   60077  loss=   13  corrupt=  0  ~ 2000.2 fps
  t+    40s  sent=   80041  recv=   80028  loss=   13  corrupt=  0  ~ 2000.1 fps
  t+    50s  sent=  100084  recv=  100072  loss=   12  corrupt=  0  ~ 2000.1 fps

======================================================================
 stress_test summary  (elapsed 60.0 s, target 2000.0 fps)
======================================================================
  Pings sent (DUT->AUX):           120025
  Pings observed at AUX:           120018
  Echos sent (AUX->DUT):           120018
  Echos observed at DUT:           120018
  Echos with corrupted payload:    0
  Unexpected frames at DUT:        0
  Unexpected frames at AUX:        0
  Rejected commands ([BELL]):      DUT=0  AUX=0
  End-to-end loss:                 7 (0.01 %)
  Actual achieved rate:            2000.1 fps

  DUT final F: F02
  AUX final F: F00
======================================================================
> python ./test/can_stress_test.py --rate 4000

=========================================================
 stress_test target devices
=========================================================
 DUT (COM9):
   slcan version: VW1K6
   serial number: N3C02
 AUX (COM8):
   slcan version: VW1K6-DEBUG
   serial number: N3C01
=========================================================
Starting: frame_type=b, S8/Y5, payload=64 bytes, rate=4000.0 fps, duration=60 s, ping_id=0x100, echo_id=0x101
Press Ctrl-C to stop early.

  t+    10s  sent=   35588  recv=   20984  loss=14604  corrupt=  0  ~ 3550.1 fps
  t+    20s  sent=   71000  recv=   41915  loss=29085  corrupt=  0  ~ 3548.7 fps
  t+    30s  sent=  106488  recv=   62892  loss=43596  corrupt=  0  ~ 3545.6 fps
  t+    40s  sent=  141857  recv=   83837  loss=58020  corrupt=  0  ~ 3542.4 fps
  t+    50s  sent=  176957  recv=  104626  loss=72331  corrupt=  0  ~ 3538.9 fps

======================================================================
 stress_test summary  (elapsed 60.0 s, target 4000.0 fps)
======================================================================
  Pings sent (DUT->AUX):           212343
  Pings observed at AUX:           125665
  Echos sent (AUX->DUT):           125665
  Echos observed at DUT:           125623
  Echos with corrupted payload:    0
  Unexpected frames at DUT:        0
  Unexpected frames at AUX:        0
  Rejected commands ([BELL]):      DUT=0  AUX=0
  End-to-end loss:                 86720 (40.84 %)
  Actual achieved rate:            3538.6 fps

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
slcan version: VW1K6
detail:
    v: hardware="USB2CANFDV1", software="2.3.0", url="github.com/Nakakiyo092/usb2canfdv1"

can port status: open/normal (1M/5Mbps)

ping: [302, 258, 331, 415, 198, 299, 201, 322, 221, 393] us


--- Stats at 0.017 hours ---

sent frames: 1454 / 1454 (-0)
  resync-detected drops: 0

status check: 1454 samples
  no error: 1454
  buffer error: 0
  can bus error: 0

timestamp comparison (host - device): 1453 samples
  ave abs error: 22.9 us
  max abs error: 181 us
  failures: 0
    of which >32767 us: 0
    of which sentinel: 0

clock accuracy: 59 sec
  clock offset: 0.0 ms
  drift upper bound: 59.1 ppm
  drift lower bound: 0.0 ppm
  (reference, time.time() based):
    host perf_counter vs wall clock: -0.1 ppm
    device drift vs wall clock:      +0.4 ppm


... snip ...


--- Stats at 3.033 hours ---

sent frames: 265763 / 265763 (-0)
  resync-detected drops: 0

status check: 265763 samples
  no error: 265763
  buffer error: 0
  can bus error: 0

timestamp comparison (host - device): 265762 samples
  ave abs error: 29.2 us
  max abs error: 4619 us
  failures: 0
    of which >32767 us: 0
    of which sentinel: 0

clock accuracy: 10919 sec
  clock offset: 0.6 ms
  drift upper bound: 0.4 ppm
  drift lower bound: 0.0 ppm
  (reference, time.time() based):
    host perf_counter vs wall clock: -0.0 ppm
    device drift vs wall clock:      +0.1 ppm

> 
```
