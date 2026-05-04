# Slcan Tester
Python test scripts for SLCAN device with CAN FD support.
- https://github.com/Nakakiyo092/usb2canfdv1
- https://github.com/Nakakiyo092/canable2-fw


## Requirements
- Python 3.13
- pyserial library (`pip install pyserial`)


## Standard test cases

1. Connect the host and the device
2. The device should not be connected to another CAN device
3. Search the device and get the device name (ex. COMX or ttyACMX)
4. Update test scripts with the device name (test/device_under_test.py)
5. Run the test script from the root directory

Windows
```
.\test\test_std_case.bat
```
Linux
```
./test/test_std_case.sh
```


## Individual test cases

1. Connect the host and the device
2. Set up CAN network following instructions for the individual test case
3. Search the device and get the device name (ex. COMX or ttyACMX)
4. Update test scripts with the device name (test/device_under_test.py)
5. Run the test script from the root directory

Windows
```
python .\test\test_slcan.py
```
```
cd test
python -m unittest test_slcan.SlcanTestCase.test_blank_command
```
Linux
```
python3 ./test/test_slcan.py
```
```
cd test
python3 -m unittest test_slcan.SlcanTestCase.test_blank_command
```
