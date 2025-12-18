# Usage of test scripts

## Usage policy

- All standard test cases must pass before releasing firmware.
- Visually confirm the pre-release checklist prior to release.
- Review individual test cases as needed before submitting a pull request.


## Standard test cases

1. Connect the host and the device
2. The device should not be connected to another CAN device
3. Search the device and get the device name (ex. COMX or ttyACMX)
4. Update test scripts with the device name
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
4. Update test scripts with the device name
5. Run the test script from the root directory

Windows
```
python .\test\test_slcan.py
```
Linux
```
python3 ./test/test_slcan.py
```


## Reference command for linux

### Search device command
```
ls /dev/tty*
```

### Set authority command
```
sudo chmod 666 /dev/ttyACM0
```

### Communicate with device
```
screen /dev/ttyACM0
```

