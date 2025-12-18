@echo off

:: Restore default setup
echo.
echo.
echo Restoring default setup
python test\tool_default_setup.py

:: Run standard test cases
echo.
echo.
echo Running slcan test cases
python test\test_slcan.py
echo.
echo.
echo Running internal loopback test cases
python test\test_in_loopback.py
echo.
echo.
echo Running external loopback test cases
python test\test_ex_loopback.py
echo.
echo.
echo Running buffer test cases
python test\test_buffer.py
echo.
echo.
echo Running error test cases
python test\test_error.py
echo.
echo.
echo Setup before reset
python test\test_reset_before.py
echo.
echo.
echo Reset the device then press enter...
pause
echo.
echo.
echo Running test after reset
python test\test_reset_after.py
echo.
echo.
echo Short CAN high and low then press enter...
pause
echo.
echo.
echo Running short test cases
python test\test_short.py
echo.
echo.
echo Make ready to check LED then press enter...
pause
echo.
echo.
echo Running led test cases...
python test\test_led.py

:: Restore default setup
echo.
echo.
echo Restoring default setup
python test\tool_default_setup.py
echo.
echo.
echo Complete

