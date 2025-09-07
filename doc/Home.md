Welcome to the USB2CANFDV1 [wiki](https://github.com/Nakakiyo092/usb2canfdv1/wiki)!

The USB2CANFDV1 device was originally developed by [WeAct Studio](https://github.com/WeActStudio/WeActStudio.USB2CANFDV1).
This document describes the behavior of an alternative firmware released [here](https://github.com/Nakakiyo092/usb2canfdv1).

This firmware implements a set of non-standard slcan commands to support CAN FD messaging alongside the LAWICEL-style command set.
The firmware enumerates as a standard serial device on Linux, macOS, and Windows for easy interfacing.
You can send or receive CAN/CAN FD frames using standard serial communication software.

For those who wish to engage deeply with this project, I strongly recommend reading the manuals for LAWICEL's CAN232 and CANUSB available on the "Useful Links" page.
These are the _**"SUTRA"**_ here.