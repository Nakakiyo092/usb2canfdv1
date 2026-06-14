# Benchmark for ver.2.1.0

Commit ID: 2a1ab7a80ced511b21792140c905d53fe277edbd

## Environment

USB Hub Usage: Yes

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
7.6.1
> python --version
Python 3.13.13
> pip freeze
pyserial==3.5
... snip ...
> Get-CimInstance Win32_USBController | Select-Object Name, Status, DeviceID

Name                                                            Status DeviceID
----                                                            ------ --------
Intel(R) USB 3.20 eXtensible Host Controller - 1.20 (Microsoft) OK     PCI\VEN_8086&DEV_A71E&SUBSYS_0BEB1028&REV_00\3&…
Intel(R) USB 3.10 eXtensible Host Controller - 1.20 (Microsoft) OK     PCI\VEN_8086&DEV_51ED&SUBSYS_0BEB1028&REV_01\3&…
USB4 (TM) ホスト ルーター (Microsoft)                           OK     PCI\VEN_8086&DEV_A73E&SUBSYS_0BEB1028&REV_00&US…
USB Composite Device                                            OK     USB\VID_FFFF&PID_BACE&REV_0002\1

> 
```

## CDC Speed Test

```powershell
> python ./test/cdc_speed_test.py COM9 --rx --chunk-size 16 --iteration 2 --duration 10
usb port name: COM9

serial number: N3C01
slcan version: VW1K4-DEBUG
detail:
    v: hardware="USB2CANFDV1", software="2.1.0", url="github.com/Nakakiyo092/usb2canfdv1"

can port status: closed

ping: [608, 577, 596, 601, 604] us

tx speed:    148.97 kB/s          1191.78 kbits/s
rx speed:    736.84 kB/s          5894.70 kbits/s
message loss:  130833  /  744864

device status: 
detail:
    z: time_ms=0xE956, time_us=0x19054672, cycle_time_us_ave_max=[0x018, 0x0ED]

tx speed:    152.03 kB/s          1216.26 kbits/s
rx speed:    741.24 kB/s          5929.92 kbits/s
message loss:  142460  /  760160

device status: 
detail:
    z: time_ms=0x279D, time_us=0x19A413C2, cycle_time_us_ave_max=[0x018, 0x0EC]

> python ./test/cdc_speed_test.py COM9 --tx --chunk-size 16 --iteration 2 --duration 10
usb port name: COM9

serial number: N3C01
slcan version: VW1K4-DEBUG
detail:
    v: hardware="USB2CANFDV1", software="2.1.0", url="github.com/Nakakiyo092/usb2canfdv1"

can port status: closed

ping: [353, 230, 236, 241, 194] us

tx speed:    529.76 kB/s          4238.05 kbits/s
rx speed:      3.72 kB/s            29.74 kbits/s
message loss:  935  /  38112

device status: 
detail:
    z: time_ms=0x9927, time_us=0x1EF31C6A, cycle_time_us_ave_max=[0x018, 0x0D8]

tx speed:    530.42 kB/s          4243.39 kbits/s
rx speed:      3.71 kB/s            29.71 kbits/s
message loss:  1024  /  38160

device status: 
detail:
    z: time_ms=0xC1CD, time_us=0x1F91E70D, cycle_time_us_ave_max=[0x017, 0x0D8]

> 
```

## Long Time Test

```powershell
> python ./test/long_time_test.py COM9 --duration 3
usb port name: COM9

serial number: N3C01
slcan version: VW1K4-DEBUG
detail:
    v: hardware="USB2CANFDV1", software="2.1.0", url="github.com/Nakakiyo092/usb2canfdv1"

can port status: open/loopback (1M/5Mbps)

ping: [514, 565, 496, 494, 478, 578, 468, 436, 416, 429] us


--- Stats at 0.017 hours ---

sent frames: 6624 / 6626 (-0)

status check: 6624 samples
  no error: 6624
  buffer error: 0
  can bus error: 0

timestamp verification: 6623 samples
  average error: 38.9 us
  max error: 775 us
  failures (>32767 us): 0

clock accuracy: 59 sec
  clock offset: -0.224 ms
  drift upper bound: 101.141 ppm
  drift lower bound: 0.000 ppm


... snip ...


--- Stats at 3.017 hours ---

sent frames: 1897192 / 1897192 (-0)

status check: 1897192 samples
  no error: 1897192
  buffer error: 0
  can bus error: 0

timestamp verification: 1897191 samples
  average error: 43.6 us
  max error: 4365 us
  failures (>32767 us): 0

clock accuracy: 10859 sec
  clock offset: -27.705 ms
  drift upper bound: 3.089 ppm
  drift lower bound: 2.013 ppm

> 
```
