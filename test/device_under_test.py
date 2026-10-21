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


    def open(self, port=None):
        """Open the connection to the device.

        Args:
            port: optional OS-specific port name override
                  (e.g., "COM8" on Windows, "/dev/ttyACM1" on Linux).
                  If None, uses the OS-default port (COM9 / /dev/ttyACM0).
        """
        # connect to serial
        if port is None:
            if sys.platform == "win32":
                port = "COM9"
            elif sys.platform.startswith("linux"):
                port = "/dev/ttyACM0"
            else:
                port = "XXX"    # TODO: put default device name in the macOS
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

        # Reset to default settings. Setup is best-effort: a firmware
        # variant may not implement every command (e.g. some CANable2.0
        # builds reject W/M/m), so an unexpected response is logged as a
        # warning rather than aborting the test. The downstream test is
        # then responsible for tolerating any leftover non-default state.
        for cmd in (b"S4\r", b"Y2\r", b"Z0\r", b"W0\r",
                    b"M00000000\r", b"mFFFFFFFF\r"):
            self.send(cmd)
            resp = self.receive()
            if resp != b"\r":
                print(f"WARNING: Setup: {cmd!r} returned {resp!r}")


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


    def send_with_stall_chain(self, tx_data: bytes, n_stalls: int = 8, stall_ms: int = 300,
                              stall_interval: float = 0.15):
        """Send frames, then a chain of stall commands (~, debug build only),
        to overflow the 3-deep HAL Tx Event FIFO and Rx FIFO of the device.
        Returns after all stalls end; the replies are left to receive().

        Use internal loopback (=) at S0, so that each frame takes several ms
        and is also received. The tx_data should hold enough frames to keep
        the HAL Tx FIFO full during the stalls (48 short frames are enough).

        The HAL Tx FIFO, the Tx Event FIFO and the Rx FIFO are all 3 deep,
        and only the main loop moves frames into the Tx FIFO and takes events
        and frames out of the other two (1 of each per iteration). So:

          1. During a stall, the 3 frames in the Tx FIFO are sent and
             received, and the Tx Event FIFO and the Rx FIFO become full.
             After the stall, the next 3 frames are moved into the Tx FIFO.
          2. If the next stall is read in the next iteration, 1 event and 1
             frame are read before it (2 left each). During the stall the 3
             frames are sent and received, but only 1 fits, so 2 are lost.

        Whether the next stall is read in the next iteration depends on when
        the USB OUT endpoint is re-armed, which is roughly a coin toss. If it
        is read later, the FIFOs are drained in between, but the stall fills
        them again, so the next pair gets another chance. A chain of stalls,
        each sent in a separate write during the previous one, makes an
        overflow practically certain. This does not depend on how many bytes
        the device handles per iteration.
        """
        stall = b"~" + format(stall_ms, "04X").encode() + b"\r"
        self.send(tx_data)
        time.sleep(0.05)    # separate the stalls from the frames, well before the frames run out
        for _ in range(0, n_stalls):
            self.send(stall)
            time.sleep(stall_interval)  # send the next stall during the previous one
        # The stalls run back to back, so wait for all of them and the remaining frames
        time.sleep(n_stalls * (stall_ms / 1000.0 - stall_interval) + 1.0)


    def receive(self) -> bytes:
        """Receive data from the device."""
        rx_data = b""
        cycle = 0.02    # sec - should be more than RRT to the device and cdc stream gap including VM USB jitter
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
