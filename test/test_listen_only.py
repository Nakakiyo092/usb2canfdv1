#!/usr/bin/env python3

import sys
import unittest

from device_under_test import DeviceUnderTest


# NOTE: All tests in this file require another device (aux) wired to the DUT
#       on the same CAN bus, with no other node on the bus. Unlike
#       test_with_receiver.py, the tests drive the aux themselves, so the aux
#       must be connected to the host at AUX_PORT and run a DEBUG build
#       (the aux opens with + or -).
#       The DUT does not ACK in listen only mode, so the aux sends with
#       + (external loopback) when the frames must succeed: the aux then
#       ignores the missing ACK and transmits each frame only once.
if sys.platform == "win32":
    AUX_PORT = "COM8"
else:
    AUX_PORT = "/dev/ttyACM1"


class ListenOnlyTestCase(unittest.TestCase):

    dut: DeviceUnderTest
    aux: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()
        self.aux = DeviceUnderTest()
        self.aux.open(AUX_PORT)
        self.aux.setup()
        if not self.aux.debug_build:
            self.aux.close()
            self.dut.close()
            self.skipTest("Requires a DEBUG build on the aux (+ and - modes)")


    def tearDown(self):
        self.aux.send(b"C\r")
        self.aux.receive()
        self.aux.close()
        self.dut.send(b"C\r")
        self.dut.receive()
        self.dut.close()


    def open_aux(self, mode: bytes):
        """Enable the Tx event report only, then open the aux with mode."""
        self.aux.send(b"z0002\r")   # no rx, tx event only
        self.assertEqual(self.aux.receive(), b"\r")
        self.aux.send(mode)
        self.assertEqual(self.aux.receive(), b"\r")


    def test_receive_in_listen_only(self):
        """Every frame type is received in listen only mode.

        The aux sends the shortest and the longest frame of each of the
        8 types with + (external loopback), so each frame succeeds without
        an ACK. The aux must report the Tx event, and the DUT must report
        the same frame.
        """
        #self.dut.print_on = True
        data = b"00112233445566778899AABBCCDDEEFF" * 4
        frames = (
            b"r03F0\r", b"t03F0\r", b"d03F0\r", b"b03F0\r",
            b"R0137FEC80\r", b"T0137FEC80\r", b"D0137FEC80\r", b"B0137FEC80\r",
            b"r03FF\r", b"t03F80011223344556677\r",
            b"d03FF" + data + b"\r", b"b03FF" + data + b"\r",
            b"R0137FEC8F\r", b"T0137FEC880011223344556677\r",
            b"D0137FEC8F" + data + b"\r", b"B0137FEC8F" + data + b"\r",
        )

        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.open_aux(b"+\r")

        for frame in frames:
            tx_event = (b"Z" if frame[:1].isupper() else b"z") + frame
            self.aux.send(frame)
            self.assertEqual(self.aux.receive(), b"\r" + tx_event,
                             f"Aux should report the Tx event of {frame!r}")
            self.assertEqual(self.dut.receive(), frame,
                             f"DUT should receive {frame!r} in listen only mode")


    def test_no_ack_in_listen_only(self):
        """The DUT does not ACK in listen only mode.

        The aux sends with - (no retransmission), so a frame without an ACK
        fails once and gives no Tx event. First the DUT opens with O and
        ACKs the frame, which confirms the wiring. Then the DUT opens with L:
        the frame must fail with the bus error (F80) on the aux, and the DUT
        must not receive it, since the aux destroys it with an error frame
        at the ACK delimiter.
        """
        #self.dut.print_on = True
        frame = b"t03F21122\r"
        self.open_aux(b"-\r")

        # Normal mode: ACKed by the DUT
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.aux.send(frame)
        self.assertEqual(self.aux.receive(), b"\r" + b"z" + frame,
                         "Aux should report the Tx event (ACKed by the DUT in normal mode)")
        self.assertEqual(self.dut.receive(), frame,
                         "DUT should receive the frame in normal mode")
        self.aux.send(b"F\r")
        self.assertEqual(self.aux.receive(), b"F00\r",
                         "Aux should see no bus error while the DUT ACKs")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Listen only mode: no ACK
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.aux.send(frame)
        self.assertEqual(self.aux.receive(), b"\r",
                         "Aux should give no Tx event (no ACK from the DUT in listen only mode)")
        self.assertEqual(self.dut.receive(), b"",
                         "DUT should not receive the frame destroyed by the ACK error")
        self.aux.send(b"F\r")
        self.assertEqual(self.aux.receive(), b"F80\r",
                         "Aux should see the bus error (ACK error)")


    def test_no_error_frame_in_listen_only(self):
        """The DUT sends no error frame in listen only mode.

        The DUT runs at Y5 (5 Mbps data) while the aux stays at Y2 (default),
        so the DUT cannot receive the data phase of b (BRS) frames, while it
        receives d (no BRS) frames. The aux sends b and d frames in turns with
        + (external loopback). In normal mode the DUT would destroy each b
        frame with an error frame. In listen only mode, every frame must
        succeed on the first try with no bus error on the aux, and the DUT
        must receive only the d frames.
        """
        #self.dut.print_on = True
        self.dut.send(b"Y5\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.open_aux(b"+\r")

        for i in range(0, 4):
            for cmd in (b"b", b"d"):
                frame = cmd + format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
                self.aux.send(frame)
                self.assertEqual(self.aux.receive(), b"\r" + b"z" + frame,
                                 f"Aux should report the Tx event of {frame!r}")
                if cmd == b"b":
                    self.assertEqual(self.dut.receive(), b"",
                                     f"DUT should not receive {frame!r} (data phase too slow)")
                else:
                    self.assertEqual(self.dut.receive(), frame,
                                     f"DUT should receive {frame!r} (no BRS)")

        self.aux.send(b"F\r")
        self.assertEqual(self.aux.receive(), b"F00\r",
                         "Aux should see no bus error (no error frame from the DUT)")


if __name__ == "__main__":
    unittest.main()
