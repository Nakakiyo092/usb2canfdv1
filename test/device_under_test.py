"""
A helper class for testing the SLCAN device under test.

License:
    MIT License.
    See the accompanying LICENSE file for full terms.
"""

import sys
import time
import serial


class DeviceUnderTest:
    """A helper class for testing the SLCAN device under test."""
    print_on: bool
    ser: serial.Serial
    slcan_ver: bytes
    debug_build: bool
    fd_support: bool

    def __init__(self):
        """Initialize the class."""
        # initialize
        self.print_on = False


    def open(self):
        """Open the connection to the device."""
        # connect to serial
        if sys.platform == "win32":
            port = "COM9"
        elif sys.platform.startswith("linux"):
            port = "/dev/ttyACM0"
        else:
            port = "XXX"
        self.ser = serial.Serial(port, timeout=1, write_timeout=1)


    def setup(self):
        """Setup the device for testing."""
        # Clear false characters in the buffer. See the link for details.
        # https://github.com/Nakakiyo092/usb2canfdv1/discussions/36
        self.send(b"\a\r\r")
        self.receive()

        # Close the CAN channel just in case
        self.send(b"C\r")
        self.receive()

        # Check the SLCAN version
        self.send(b"V\r")
        slcan_ver = self.receive()
        if slcan_ver[:4] == b"VL2K":
            # CANable2.0 "Nakakiyo092/canable2-fw"
            self.fd_support = True
        elif slcan_ver[:4] == b"VW1K":
            # WeAct Studio "Nakakiyo092/usb2canfdv1"
            self.fd_support = True
        else:
            self.fd_support = False
            print("WARNING: Unsupported SLCAN version ", slcan_ver.decode())

        self.debug_build = bool(b"DEBUG" in slcan_ver)

        # Reset to default settings
        self.send(b"S4\r")
        assert self.receive() == b"\r", "Setup: S4 command failed"
        self.send(b"Y2\r")
        assert self.receive() == b"\r", "Setup: Y2 command failed"
        self.send(b"Z0\r")
        assert self.receive() == b"\r", "Setup: Z0 command failed"
        self.send(b"W0\r")
        assert self.receive() == b"\r", "Setup: W0 command failed"
        self.send(b"M00000000\r")
        assert self.receive() == b"\r", "Setup: M command failed"
        self.send(b"mFFFFFFFF\r")       # mFFFFFFFF -> Pass all
        assert self.receive() == b"\r", "Setup: m command failed"


    def close(self):
        """Close the connection to the device."""
        # close serial
        self.ser.close()


    def print_data(self, direction: str, data: bytes):
        """Print the data in a human-readable format.
        param direction: 'T' for transmit, 'R' for receive
        """
        datar = data
        datar = datar.replace(b"\r", b"[CR]")
        datar = datar.replace(b"\a", b"[BELL]")
        if direction in ('t', 'T'):
            print("")
            print("<<< ", datar.decode())
        else:
            print("")
            print(">>> ", datar.decode())


    def send(self, tx_data: bytes):
        """Send data to the device."""
        self.ser.write(tx_data)

        if self.print_on:
            self.print_data("T", tx_data)


    def receive(self) -> bytes:
        """Receive data from the device."""
        rx_data = b""
        cycle = 0.02    # sec
        timeout = 1     # sec
        for _ in range(0, int(timeout / cycle)):
            time.sleep(cycle)
            tmp = self.ser.read_all()
            rx_data = rx_data + tmp
            if len(tmp) == 0 and len(rx_data) != 0:
                break

        if self.print_on:
            self.print_data("R", rx_data)

        return rx_data
