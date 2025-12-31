import time
import serial


class DeviceUnderTest:

    print_on: bool
    ser: serial
    slcan_ver: bytes

    def __init__(self):
        # initialize
        self.print_on = False


    def open(self):
        # connect to serial
        # device name should be changed
        #self.ser = serial.Serial('/dev/ttyACM0', timeout=1, write_timeout=1)
        self.ser = serial.Serial('COM9', timeout=1, write_timeout=1)


    def setup(self):
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
            # CANable2.0
            pass
        elif slcan_ver[:4] == b"VW1K":
            # WeAct Studio
            pass
        else:
            print("NOTE: Unknown SLCAN version ", slcan_ver.decode())

        # Reset to default settings
        self.send(b"S4\r")
        self.receive()
        self.send(b"Y2\r")
        self.receive()
        self.send(b"Z0\r")
        self.receive()
        self.send(b"W0\r")
        self.receive()
        self.send(b"M00000000\r")
        self.receive()
        self.send(b"mFFFFFFFF\r")       # mFFFFFFFF -> Pass all
        self.receive()


    def close(self):
        # close serial
        self.ser.close()


    def print_data(self, dir: chr, data: bytes):
        datar = data
        datar = datar.replace(b"\r", b"[CR]")
        datar = datar.replace(b"\a", b"[BELL]")
        if dir == "t" or dir == "T":
            print("")
            print("<<< ", datar.decode())
        else:
            print("")
            print(">>> ", datar.decode())


    def send(self, tx_data: bytes):
        self.ser.write(tx_data)

        if (self.print_on):
            self.print_data("T", tx_data)


    def receive(self) -> bytes:
        rx_data = b""
        cycle = 0.02    # sec
        timeout = 1     # sec
        for i in range(0, int(timeout / cycle)):
            time.sleep(cycle)
            tmp = self.ser.read_all()
            rx_data = rx_data + tmp
            if len(tmp) == 0 and len(rx_data) != 0:
                break

        if (self.print_on):
            self.print_data("R", rx_data)
        
        return rx_data

