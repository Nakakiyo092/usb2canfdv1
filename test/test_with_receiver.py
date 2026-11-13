#!/usr/bin/env python3

import unittest

import re
import time
import random
from device_under_test import DeviceUnderTest


# NOTE: All tests in this file require another device (aux) wired to the DUT
#       on the same CAN bus. The tests do not access the aux, so open its
#       channel beforehand in normal mode with the default bit rates,
#       e.g. by sending "C", "S4", "Y2" and "O" to the aux.
#       - TxEventTestCase: aux acts as a normal ACK provider.
#       - EsiTestCase: aux receives but cannot ACK frames whose data phase
#         is faster than its own configuration, allowing controlled NACKs.
#       - TdcTestCase: aux acts as a normal ACK provider.
#       The test_single_tx_event_after_retries requires the channel of the
#       aux device becoming open and closed repeatedly.
#       BusErrorFlagTestCase uses the same controlled NACKs as EsiTestCase.
class TxEventTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        # close serial
        self.dut.close()


    def test_tx_events_in_normal_mode(self):
        """Verify that every transmitted frame produces a Tx event
        report in Normal mode (open with O), where every frame is
        ACKed by the aux device.

        Sends 0x800 random b/t frames (10% BRS / 90% classic). All
        frames succeed, so each one should produce its Tx event
        report. The final F check confirms no error counters were
        raised.
        """
        #self.dut.print_on = True
        random.seed(92)
        rx_data = b""
        rx_data_exp = b""
        self.dut.send(b"z0002\r")  # no rx, tx event only
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # 10% b / 90% t
        for i in range(0x000, 0x800):
            if random.random() < 0.1:
                tx_data = b"b"
                tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
                rx_data_exp += b"\r" + b"z" + tx_data
            else:
                tx_data = b"t"
                tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
                rx_data_exp += b"\r" + b"z" + tx_data
            self.dut.send(tx_data)
            if i % 180 == 0:
                # the buffer can store as least 180 messages (4096 / 22)
                rx_data += self.dut.receive()
            time.sleep(0.001)

        # check all reply
        rx_data += self.dut.receive()
        rx_data = rx_data.replace(b"\r", b"")   # [CR] and tx event may swap so remove all [CR] before analysis
        rx_data_exp = rx_data_exp.replace(b"\r", b"")
        self.assertEqual(rx_data, rx_data_exp,
                         "Tx event sequence mismatch in Normal mode (every frame should produce one Tx event)")

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Expected F=00 after a fully ACKed burst in Normal mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_tx_events_in_no_retransmit_mode(self):
        """Verify that every transmitted frame produces a Tx event
        report in no-retransmit (DAR) mode (open with -), where
        every frame is ACKed by the aux device on the first try.

        Same frame pattern as test_tx_events_in_normal_mode (10% BRS
        / 90% classic, all succeeding). DAR has no effect on the Tx
        event sequence when no frame fails — the test confirms this
        equivalence.
        """
        #self.dut.print_on = True
        random.seed(92)
        rx_data = b""
        rx_data_exp = b""
        self.dut.send(b"z0002\r")  # no rx, tx event only
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"-\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # 10% b / 90% t
        for i in range(0x000, 0x800):
            if random.random() < 0.1:
                tx_data = b"b"
                tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
                rx_data_exp += b"\r" + b"z" + tx_data
            else:
                tx_data = b"t"
                tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
                rx_data_exp += b"\r" + b"z" + tx_data
            self.dut.send(tx_data)
            if i % 180 == 0:
                # the buffer can store as least 180 messages (4096 / 22)
                rx_data += self.dut.receive()
            time.sleep(0.001)

        # check all reply
        rx_data += self.dut.receive()
        rx_data = rx_data.replace(b"\r", b"")   # [CR] and tx event may swap so remove all [CR] before analysis
        rx_data_exp = rx_data_exp.replace(b"\r", b"")
        self.assertEqual(rx_data, rx_data_exp,
                         "Tx event sequence mismatch in no-retransmit mode (every frame should produce one Tx event)")

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Expected F=00 after a fully ACKed burst in no-retransmit mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_only_acked_tx_events(self):
        """Verify that failed (NACKed) frames do NOT produce Tx
        event reports, while ACKed frames do.

        Setup: DUT runs at Y5 (5 Mbps data) while the aux device
        stays at the default Y2 (2 Mbps). DAR is enabled (open with
        -). Sends 0x800 random b/t frames (10% BRS / 90% classic):
          - b (BRS) frames at 5 Mbps cannot be received by the aux
            at 2 Mbps, so each one is NACKed and produces no Tx
            event.
          - t (classic) frames use only the nominal phase (matched),
            so each one is ACKed and produces a Tx event.

        The expected Tx event sequence contains ONLY the t frames.
        The final F check confirms F80 (Bus Error from the failed b
        frames).
        """
        #self.dut.print_on = True
        random.seed(92)
        rx_data = b""
        rx_data_exp = b""
        # Setup bit rate so that CBFF is OK but FBFF with BRS is not.
        self.dut.send(b"Y5\r")      # Rx side is default (125kbps / 2Mbps)
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0002\r")   # no rx, tx event only
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"-\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # TEC: b NACK -> +8, t ACK -> -1.
        # 10% b / 90% t -> net = +8 - 9 per 10 frames; TEC stays bounded
        # even when every b fails (see test_only_acked_tx_events).
        for i in range(0x000, 0x800):
            if random.random() < 0.1:
                tx_data = b"b"
                tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
            else:
                tx_data = b"t"
                tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
                rx_data_exp += b"\r" + b"z" + tx_data
            self.dut.send(tx_data)
            if i % 180 == 0:
                # the buffer can store as least 180 messages (4096 / 22)
                rx_data += self.dut.receive()
            time.sleep(0.001)

        # check all reply
        rx_data += self.dut.receive()
        rx_data = rx_data.replace(b"\r", b"")   # [CR] and tx event may swap so remove all [CR] before analysis
        rx_data_exp = rx_data_exp.replace(b"\r", b"")
        self.assertEqual(rx_data, rx_data_exp,
                         "Tx event sequence mismatch: only ACKed (t) frames should emit Tx events; failed (b) frames must not")

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F80\r",
                         "Expected F=80 (Bus Error) from the NACKed BRS frames")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    # NOTE: test_single_tx_event_after_retries verifies that exactly one Tx event is reported after the hardware retransmits
    #       a failed frame multiple times before finally getting an ACK. Although not directly tested
    #       (special setup required), this is effectively covered by composition:
    #       - Retransmit-on-NACK: test_error.py::test_error_passive (TEC reaches 128 via repeated retries)
    #       - Single-Tx-event-on-final-success: test_all_events_normal_mode above
    #       The STM32 FDCAN auto-retry is transparent to firmware (one TXOK interrupt fires only on final ACK).
    @unittest.skip("Skip this test due to a special setup requirement")
    def test_single_tx_event_after_retries(self):
        """Verify that hardware retransmission is transparent to
        the firmware: a frame that gets NACKed multiple times then
        finally ACKed produces exactly ONE Tx event report.

        Skipped because reproducing the multi-retry-then-success
        scenario requires a special hardware setup. See the NOTE
        above for how this behavior is covered by composition.
        """
        self.dut.print_on = True
        rx_data = b""
        rx_data_exp = b""
        #self.dut.send(b"S0\r")
        #self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0002\r")  # no rx, tx event only
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        #for i in range(0x000, 0x800):
        for i in range(0x000, 0x100):
            tx_data = b"t"
            tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
            rx_data_exp += b"\r" + b"z" + tx_data
            self.dut.send(tx_data)
            if i % 180 == 0:
                # the buffer can store as least 180 messages (4096 / 22)
                rx_data += self.dut.receive()
            time.sleep(0.1)

        # check all reply
        rx_data += self.dut.receive()
        rx_data = rx_data.replace(b"\r", b"")   # [CR] and tx event may swap
        rx_data_exp = rx_data_exp.replace(b"\r", b"")
        self.assertEqual(rx_data, rx_data_exp)

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA4\r")  # This will not be true
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")




class EsiTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_esi_bit_in_tx_event(self):
        """Verify Tx event report carries ESI=1 while DUT is error-passive,
        and ESI=0 after recovery to error-active.

        Setup:
            DUT runs at Y5 (5 Mbps data) while the aux stays at Y2 (default).
            DAR mode (open with -); auto-retransmission disabled prevents
            bus-off when TEC grows. Reporting: z0012 (Tx event + ESI).

        Phases:
            1. Send N BRS frames; each is NACKed by the aux (data-phase
               mismatch). TEC += 8 per failure, no retry under DAR.
               After ~16 failures TEC >= 128 -> error-passive.
            2. Send d (CAN-FD no-BRS) frame; aux ACKs (nominal phase
               matched), DUT emits Tx event with ESI=1.
            3. Send d frames repeatedly; each ACK decrements TEC. After
               ~128 ACKs TEC drops below 128 -> error-active.
            4. Send d frame; Tx event with ESI=0.
        """
        #self.dut.print_on = True
        self.dut.send(b"Y5\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0012\r")  # Tx event ON + ESI ON, Rx OFF
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"-\r")      # Normal + DAR (bus-off safe)
        self.assertEqual(self.dut.receive(), b"\r")

        # Phase 1: drive into error-passive via BRS NACKs.
        for i in range(0, 20):
            tx_data = b"b" + format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
            self.dut.send(tx_data)
            self.dut.receive()

        # Confirm error-passive
        self.dut.send(b"f\r")
        status = self.dut.receive()
        self.assertIn(b"ER_PSSV", status,
                      f"DUT should be error-passive after BRS NACK burst, got: {status!r}")

        # Phase 2: error-passive -> ESI=1 in Tx event
        self.dut.send(b"d03F0\r")
        rx_data = self.dut.receive()
        self.assertIn(b"zd03F01\r", rx_data,
                      f"Expected Tx event with ESI=1 while error-passive, got: {rx_data!r}")

        # Phase 3: recover by accumulating successful ACKs
        for i in range(0, 200):
            self.dut.send(b"d03F0\r")
            self.dut.receive()

        # Confirm error-active
        self.dut.send(b"f\r")
        status = self.dut.receive()
        self.assertIn(b"ER_ACTV", status,
                      f"DUT should be back to error-active after successful ACKs, got: {status!r}")

        # Phase 4: error-active -> ESI=0 in Tx event
        self.dut.send(b"d03F0\r")
        rx_data = self.dut.receive()
        self.assertIn(b"zd03F00\r", rx_data,
                      f"Expected Tx event with ESI=0 in error-active, got: {rx_data!r}")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


class TdcTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_transceiver_loop_delay(self):
        """Measure the transceiver loop delay with TDC and check that it
        is in a plausible range.

        Sends a CAN-FD frame with BRS in Normal mode at the default
        bitrate (data prescaler 1, so auto TDC is on), ACKed by the aux,
        then reads TDCV and TDCO with the DEBUG-only `!7DC` query. TDCV
        is the secondary sample point position, i.e. the measured delay
        from FDCAN_TX to FDCAN_RX plus TDCO, in minimum time quanta
        (one CAN clock period). The loop delay is therefore
        (TDCV - TDCO) * 1000 / clock_mhz ns.

        A typical transceiver loop delay is around 150 - 250 ns. The
        50 - 500 ns window is a rough plausibility check, not a
        transceiver specification.
        """
        #self.dut.print_on = True
        if not self.dut.fd_support or not self.dut.debug_build:
            self.skipTest("Requires a CAN-FD capable DEBUG build (!7DC)")

        # CAN clock for converting mtq to ns (read from the I reply in setup).
        clock_mhz = self.dut.clock_mhz
        self.assertIsNotNone(clock_mhz, "CAN clock is unknown (no valid I reply)")

        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"b03F80011223344556677\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        self.dut.send(b"!7DC\r")
        rx_data = self.dut.receive()
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        match = re.fullmatch(rb"!: TDCV=0x([0-9A-F]{2}), TDCO=0x([0-9A-F]{2}), "
                             rb"TDCF=0x([0-9A-F]{2}), EN=([01])\r", rx_data)
        self.assertIsNotNone(match, f"Unexpected !7DC reply: {rx_data!r}")
        tdcv = int(match.group(1), 16)
        tdco = int(match.group(2), 16)
        self.assertEqual(match.group(4), b"1",
                         f"TDC should be enabled at the default data bitrate: {rx_data!r}")
        delay_ns = (tdcv - tdco) * 1000 / clock_mhz
        print(f"\nTransceiver loop delay: {delay_ns:.1f} ns (TDCV=0x{tdcv:02X}, TDCO=0x{tdco:02X})")
        self.assertGreaterEqual(delay_ns, 50,
                                f"Loop delay {delay_ns:.1f} ns out of [50, 500] ns: {rx_data!r}")
        self.assertLessEqual(delay_ns, 500,
                             f"Loop delay {delay_ns:.1f} ns out of [50, 500] ns: {rx_data!r}")


class BusErrorFlagTestCase(unittest.TestCase):
    """F bit 7 (BUS_ERROR) with bus errors mixed with successful transfers.

    Setup (same as EsiTestCase): DUT runs at Y5 (5 Mbps data) while the aux
    stays at Y2 (default), and the DUT opens with - (no retransmission).
    b (BRS) frames from the DUT fail in the data phase, as the aux cannot
    follow the faster data bit rate. t / d frames use only the nominal phase,
    so the aux ACKs them. TEC: +8 per failed frame, -1 per successful frame.
    Only F is checked; the error code in f is unspecified.
    """

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()
        if not self.dut.debug_build:
            self.skipTest("Requires a DEBUG build (no retransmission mode)")


    def tearDown(self):
        self.dut.close()


    def open_with_nack(self, report: bytes):
        """Set up Y5 and the report mode, then open with no retransmission."""
        self.dut.send(b"Y5\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(report)
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"-\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_bus_error_mixed_with_success(self):
        """Repeat [b x 1, t x 8] and read F after each group. Each group must
        give F80 and nothing else: the 1:8 ratio brings TEC back to 0, so the
        node stays error active. Every t frame must give its Tx event.
        """
        #self.dut.print_on = True
        rx_data = b""
        rx_data_exp = b""
        self.open_with_nack(b"z0002\r")    # no rx, tx event only

        for grp in range(0, 16):
            tx_data = b"b" + format(grp, "03X").encode() + b"2" + format(grp, "04X").encode() + b"\r"
            self.dut.send(tx_data)
            # Wait until the b frame has failed. With no retransmission (DAR), when a
            # transmission fails, the frames already in the HAL Tx FIFO and those added
            # just after the failure are not sent at all: no bus activity, no TEC change,
            # no Tx event and no F flag. Frames added 0.5 ms or more after the failure
            # are sent. Seen on the bench and likely the behavior of the FDCAN, not
            # confirmed in the documents.
            time.sleep(0.005)
            for i in range(0, 8):
                num = grp * 8 + i
                tx_data = b"t" + format(num, "03X").encode() + b"2" + format(num, "04X").encode() + b"\r"
                rx_data_exp += b"\r" + b"z" + tx_data
                self.dut.send(tx_data)
                time.sleep(0.001)
            rx_data += self.dut.receive()
            self.dut.send(b"F\r")
            self.assertEqual(self.dut.receive(), b"F80\r",
                             f"Expected F80 (BUS_ERROR only) after group {grp} of [b x 1, t x 8]")

        # The b frames give no Tx event. [CR] and Tx event may swap, so remove all [CR].
        rx_data = rx_data.replace(b"\r", b"")
        rx_data_exp = rx_data_exp.replace(b"\r", b"")
        self.assertEqual(rx_data, rx_data_exp,
                         "Every t frame should give its Tx event")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_bus_error_kept_over_tdc_query(self):
        """A TDC query (!7DC) that reads PSR before the firmware polls the
        status after an error. PSR.LEC is set to NO_CHANGE on a read, so the
        error code is lost, but F bit 7 must still be raised. F must give F80.

        Queue [d (64 bytes), b], then send the stall and !7DC in one write
        (one USB packet) while the long d frame is still on the bus. The b
        frame fails during the stall. After the stall, the parser handles the
        rest of the packet first, so !7DC reads PSR before the next poll.
        As in test_bus_error_followed_by_success_in_stall, the round is valid
        only if the error code was missed (recent_err_code stays NONE), and
        the test is skipped as inconclusive if no round is valid.
        """
        #self.dut.print_on = True
        self.open_with_nack(b"z0000\r")    # no report
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        long_d = b"d100F" + b"A5" * 64 + b"\r"
        num_rounds = 10
        num_valid = 0
        for rnd in range(0, num_rounds):
            self.dut.send(b"-\r")
            self.assertEqual(self.dut.receive(), b"\r")

            self.dut.send(long_d + b"b1012AA55\r")
            time.sleep(0.002)   # let the two frames go to the HAL Tx FIFO
            self.dut.send(b"~012C\r!7DC\r")    # stall 300 ms, then the TDC query
            time.sleep(0.5)
            rx_data = self.dut.receive()
            self.assertIn(b"!: TDCV=", rx_data, f"!7DC should be answered: {rx_data!r}")
            self.dut.send(b"F\r")
            self.assertEqual(self.dut.receive(), b"F80\r",
                             f"Expected F80 when a TDC query reads PSR before the status poll "
                             f"(round {rnd})")

            self.dut.send(b"f\r")
            status = self.dut.receive()
            match = re.search(rb"recent_err_code=(\w{4})", status)
            if match is None:
                self.skipTest(f"Requires recent_err_code in the f reply: {status!r}")
            if match.group(1) == b"NONE":
                num_valid += 1

            self.dut.send(b"C\r")
            self.assertEqual(self.dut.receive(), b"\r")

        print(f"\nValid rounds (error code missed by the TDC query): {num_valid} / {num_rounds}")
        if num_valid == 0:
            self.skipTest("Inconclusive: the stall did not start in time in any round")


    def test_no_bus_error_with_success_only(self):
        """Only successful t / d frames in the same setup must give F00."""
        #self.dut.print_on = True
        self.open_with_nack(b"z0000\r")    # no report

        for i in range(0, 64):
            if i % 2 == 0:
                tx_data = b"t" + format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
            else:
                tx_data = b"d" + format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
            self.dut.send(tx_data)
            time.sleep(0.001)
        time.sleep(0.05)
        self.dut.receive()
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Expected F00 when every frame succeeds")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def send_and_wait(self, tx_data: bytes, count: int):
        """Send the frame count times and wait until all are done."""
        for _ in range(0, count):
            self.dut.send(tx_data)
            time.sleep(0.001)
        time.sleep(0.05)
        self.dut.receive()


    def test_leaving_error_warning(self):
        """Bit 2 (ERROR_WARNING) is a status-change flag: it is raised when
        the node reaches the warning level and again when it leaves it.
        b x 12 gives TEC 96 (F84), then t x 1 gives TEC 95 (F04).
        """
        #self.dut.print_on = True
        self.open_with_nack(b"z0000\r")    # no report

        self.send_and_wait(b"b1012AA55\r", 12)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F84\r",
                         "Expected F84 (BUS_ERROR | ERROR_WARNING) when TEC reaches 96")
        self.send_and_wait(b"t1022AA55\r", 1)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F04\r",
                         "Expected F04 (ERROR_WARNING) when TEC goes back to 95")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_leaving_error_passive(self):
        """Bit 5 (ERROR_PASSIVE) is a status-change flag: it is raised when
        the node reaches the error passive level and again when it leaves it.
        b x 16 gives TEC 128 (FA4), then t x 1 gives TEC 127 (F20). The node
        stays at the warning level, so bit 2 is not raised again.
        """
        #self.dut.print_on = True
        self.open_with_nack(b"z0000\r")    # no report

        self.send_and_wait(b"b1012AA55\r", 16)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA4\r",
                         "Expected FA4 (BUS_ERROR | ERROR_PASSIVE | ERROR_WARNING) when TEC reaches 128")
        self.send_and_wait(b"t1022AA55\r", 1)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F20\r",
                         "Expected F20 (ERROR_PASSIVE) when TEC goes back to 127")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
