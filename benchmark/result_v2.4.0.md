# Benchmark for ver.2.4.0

Commit ID: 6bdec776cd2e42590bb356d2c1768ba175406c56

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
7.6.7
> python --version
Python 3.13.14
> pip freeze
pyserial==3.5
> 
```

## CDC Speed Test

```powershell
> python ./test/cdc_speed_test.py COM8 --rx --chunk-size 16 --iteration 2 --duration 10
usb port name: COM8

serial number: N3C02
slcan version: VW1K7
detail:
    v: hardware="USB2CANFDV1", software="2.4.0", url="github.com/Nakakiyo092/usb2canfdv1"

can port status: closed

ping: [530, 437, 205, 357, 247] us

tx speed:    174.61 kB/s          1396.86 kbits/s
rx speed:    439.51 kB/s          3516.05 kbits/s
message loss:  140529  /  873040

device status: 
detail:
    z: time_ms=0x1B33, time_us=0x0B24D659, cycle_time_us_ave_max=[0x015, 0x0FB]

tx speed:    162.01 kB/s          1296.05 kbits/s
rx speed:    427.50 kB/s          3419.98 kbits/s
message loss:  97537  /  810032

device status: 
detail:
    z: time_ms=0x43D9, time_us=0x0BC3A05F, cycle_time_us_ave_max=[0x015, 0x0FD]

> python ./test/cdc_speed_test.py COM8 --tx --chunk-size 16 --iteration 2 --duration 10
usb port name: COM8

serial number: N3C02
slcan version: VW1K7
detail:
    v: hardware="USB2CANFDV1", software="2.4.0", url="github.com/Nakakiyo092/usb2canfdv1"

can port status: closed

ping: [387, 425, 449, 465, 446] us

tx speed:    536.43 kB/s          4291.43 kbits/s
rx speed:      2.71 kB/s            21.68 kbits/s
message loss:  11488  /  38592

device status: 
detail:
    z: time_ms=0xC759, time_us=0x0DC54BF5, cycle_time_us_ave_max=[0x015, 0x0FF]

tx speed:    543.10 kB/s          4344.81 kbits/s
rx speed:      2.69 kB/s            21.54 kbits/s
message loss:  12145  /  39072

device status: 
detail:
    z: time_ms=0x05A2, time_us=0x0E641CD4, cycle_time_us_ave_max=[0x015, 0x0FF]

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
 S0   .   .   E   -   P   P   -   -   -   - 
 S1   .   .   .   -   O   P   -   -   -   - 
 S2   .   .   .   -   P   O   -   -   -   - 
 S3   .   .   .   -   .   .   -   -   -   - 
 S4   .   .   .   -   .   .   -   -   -   - 
 S5   .   .   .   -   .   .   -   -   -   - 
 S6   .   .   .   -   .   .   -   -   -   - 
 S7   .   .   .   -   .   .   -   -   -   - 
 S8   .   .   .   -   .   .   -   -   -   - 
 S9   -   -   -   -   -   -   -   -   -   - 

summary:  pass=38  busErr=1  passive=4  busOff=2  skip=55  other=0
================================================================
.
================================================================
test_sp_grid_report  (nominal S8, rows = data SP, columns = data bit rate)
CAN clock: DUT 80 MHz, AUX 80 MHz
Legend: .=pass  E=bus_error  P=passive  O=bus-off  -=not representable  ?=other

         1M   2M   4M   5M   8M  10M  16M  20M
  20%    .    .    .    -    P    -    -    - 
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

summary:  pass=63  busErr=0  passive=1  busOff=3  skip=61  other=0
================================================================
.
----------------------------------------------------------------------
Ran 3 tests in 360.272s

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
   slcan version: VW1K7-DEBUG
   serial number: N3C01
 AUX (COM8):
   slcan version: VW1K7
   serial number: N3C02
=========================================================
Starting: frame_type=b, S8/Y5, payload=64 bytes, rate=2000.0 fps, duration=60 s, ping_id=0x100, echo_id=0x101
Press Ctrl-C to stop early.

  t+    10s  sent=   20069  recv=   19654  pending=  415  corrupt=  0  ~ 2000.4 fps
  t+    20s  sent=   40017  recv=   39169  pending=  848  corrupt=  0  ~ 2000.2 fps
  t+    30s  sent=   60064  recv=   58691  pending= 1373  corrupt=  0  ~ 2000.2 fps
  t+    40s  sent=   80010  recv=   78122  pending= 1888  corrupt=  0  ~ 2000.1 fps
  t+    50s  sent=  100050  recv=   97690  pending= 2360  corrupt=  0  ~ 2000.0 fps

======================================================================
 stress_test summary  (elapsed 60.0 s, target 2000.0 fps)
======================================================================
  Pings sent (DUT->AUX):           120005
  Pings observed at AUX:           118346
  Echos sent (AUX->DUT):           118346
  Echos observed at DUT:           117142
  Echos with corrupted payload:    0
  Unexpected frames at DUT:        0
  Unexpected frames at AUX:        0
  Rejected commands ([BELL]):      DUT=0  AUX=0
  End-to-end loss:                 2863 (2.39 %)
  Actual achieved rate:            2000.0 fps

  DUT final F: F02
  AUX final F: F02
======================================================================
> python ./test/can_stress_test.py --rate 4000

=========================================================
 stress_test target devices
=========================================================
 DUT (COM9):
   slcan version: VW1K7-DEBUG
   serial number: N3C01
 AUX (COM8):
   slcan version: VW1K7
   serial number: N3C02
=========================================================
Starting: frame_type=b, S8/Y5, payload=64 bytes, rate=4000.0 fps, duration=60 s, ping_id=0x100, echo_id=0x101
Press Ctrl-C to stop early.

  t+    10s  sent=   35074  recv=   15556  pending=19518  corrupt=  0  ~ 3499.2 fps
  t+    20s  sent=   70290  recv=   31107  pending=39183  corrupt=  0  ~ 3506.3 fps
  t+    30s  sent=  105379  recv=   46501  pending=58878  corrupt=  0  ~ 3510.8 fps
  t+    40s  sent=  140741  recv=   61953  pending=78788  corrupt=  0  ~ 3515.6 fps
  t+    50s  sent=  176191  recv=   77236  pending=98955  corrupt=  0  ~ 3523.5 fps

======================================================================
 stress_test summary  (elapsed 60.0 s, target 4000.0 fps)
======================================================================
  Pings sent (DUT->AUX):           211469
  Pings observed at AUX:           93065
  Echos sent (AUX->DUT):           93065
  Echos observed at DUT:           92786
  Echos with corrupted payload:    0
  Unexpected frames at DUT:        0
  Unexpected frames at AUX:        0
  Rejected commands ([BELL]):      DUT=0  AUX=0
  End-to-end loss:                 118683 (56.12 %)
  Actual achieved rate:            3522.8 fps

  DUT final F: 
  AUX final F: F02
======================================================================
> 
```

## Long Time Test

```powershell
> python ./test/long_time_test.py COM8 --duration 3 --with-receiver
usb port name: COM8

serial number: N3C02
slcan version: VW1K7
detail:
    v: hardware="USB2CANFDV1", software="2.4.0", url="github.com/Nakakiyo092/usb2canfdv1"

can port status: open/normal (1M/5Mbps)

ping: [306, 261, 257, 247, 246, 259, 250, 241, 257, 164] us


--- Stats at 0.017 hours ---

sent frames: 1440 / 1440 (-0)
  resync-detected drops: 0

status check: 1440 samples
  no error: 1440
  buffer error: 0
  can bus error: 0

timestamp comparison (host - device): 1439 samples
  ave abs error: 34.6 us
  max abs error: 135 us
  failures: 0
    of which >32767 us: 0
    of which sentinel: 0

clock accuracy: 59 sec
  clock offset: 0.2 ms
  drift upper bound: 52.8 ppm
  drift lower bound: 0.0 ppm
  (reference, time.time() based):
    host perf_counter vs wall clock: -0.0 ppm
    device drift vs wall clock:      +2.7 ppm


... snip ...


--- Stats at 3.033 hours ---

sent frames: 268039 / 268039 (-0)
  resync-detected drops: 0

status check: 268039 samples
  no error: 268039
  buffer error: 0
  can bus error: 0

timestamp comparison (host - device): 268038 samples
  ave abs error: 26.6 us
  max abs error: 16356 us
  failures: 0
    of which >32767 us: 0
    of which sentinel: 0

clock accuracy: 10919 sec
  clock offset: 19.8 ms
  drift upper bound: 2.1 ppm
  drift lower bound: 1.5 ppm
  (reference, time.time() based):
    host perf_counter vs wall clock: -0.0 ppm
    device drift vs wall clock:      +1.8 ppm

> 
```
