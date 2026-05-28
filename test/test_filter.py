#!/usr/bin/env python3
"""
Test cases for acceptance filter modes.

W0 (dual filter mode): The 4-byte Code (M command) and Mask (m command)
are split into two independent 2-byte filters:
  - Filter 1: bytes AC0/AC1 (most-significant pair)
  - Filter 2: bytes AC2/AC3 (least-significant pair)
A frame is accepted if EITHER filter matches (logical OR).

Mapping for base (standard) CAN ID (11 bits) in dual mode:
  AC0/AM0[7:0] -> ID[10:3],  AC1/AM1[7:5] -> ID[2:0]
  AC1/AM1[4:0]: accepted as input but not compared (".")
  Filter 2 uses AC2/AM2 and AC3/AM3 with the same mapping.

Mapping for extended CAN ID (29 bits) in dual mode:
  AC0/AM0[7:0] -> ID[28:21],  AC1/AM1[7:0] -> ID[20:13]
  ID[12:0]: always don't-care
  Filter 2 uses AC2/AM2 and AC3/AM3 with the same mapping.

W2 (simple filter mode): The full 4-byte Code (M command) and Mask (m
command) form a single filter. A frame is accepted only if it matches
that filter.

Mapping for base (standard) CAN ID (11 bits) in simple mode:
  M[31] selects frame type: 1=STD only, 0=EXT only, ignored when m[31]=1
  M[28:21] -> ID[10:3],  M[20:18] -> ID[2:0]
  M[17:13]: accepted as input but not compared (".")
  M[12:0] -> duplicated lower bits (same mapping as M[28:16])

Mapping for extended CAN ID (29 bits) in simple mode:
  M[28:0] -> ID[28:0] (exact 29-bit mapping)

Reference: doc/3.-Acceptance-Filter.md
"""

import unittest

from device_under_test import DeviceUnderTest


class DualFilterTestCase(unittest.TestCase):
    """Test cases for the W0 dual acceptance filter mode."""

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()
        # setUp already sets: W0 + M00000000 + mFFFFFFFF (pass-all dual filter)

    def tearDown(self):
        self.dut.close()


    def test_pass_all(self):
        """W0 with mFFFFFFFF (all bits don't-care) passes all standard and extended frames."""
        # setUp has already configured W0 + M00000000 + mFFFFFFFF
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Standard frames: a representative sample across the ID range
        for std_id in (0x000, 0x001, 0x100, 0x3FF, 0x601, 0x7FE, 0x7FF):
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"\r",
                             f"STD ID {std_id:#x} should pass with mFFFFFFFF")

        # Extended frames: a representative sample across the ID range
        for ext_id in (0x00000000, 0x00000001, 0x0137FEC8, 0x18DA0000, 0x1FFFFFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"\r",
                             f"EXT ID {ext_id:#x} should pass with mFFFFFFFF")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_same_std_id_both_filters(self):
        """
        Doc Example 2: both filters configured to accept STD ID 0x601 only.
        Code=C03FC03F, Mask=001F001F

        Bit calculation:
          AC0=0xC0, AC1=0x3F  ->  code_std_f1 = (0xC0<<3)|(0x3F>>5) = 0x600|0x01 = 0x601
          AM0=0x00, AM1=0x1F  ->  mask_std_f1 = (0x00<<3)|(0x1F>>5) = 0x000 (exact match)
          AC2=0xC0, AC3=0x3F  ->  code_std_f2 = 0x601  (same as Filter 1)

        EXT: incidentally accepts 0x18040000..0x1807FFFF (both filters).
        """
        self.dut.send(b"MC03FC03F\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m001F001F\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # STD ID 0x601 must pass (matched by both Filter 1 and Filter 2)
        self.dut.send(b"t6010\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t6010\r")

        # Adjacent and other STD IDs must be blocked
        for std_id in (0x600, 0x602, 0x401, 0x001, 0x000, 0x7FF):
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD ID {std_id:#x} should be blocked")

        # EXT IDs in incidental pass range 0x18040000..0x1807FFFF must pass
        #   code_ext_f1 = (0xC0<<21)|(0x3F<<13) = 0x1807E000
        #   mask_ext_f1 = (0x00<<21)|(0x1F<<13)|0x1FFF = 0x0003FFFF
        #   STM32 mask = 0x1FFC0000 -> accepted: (ID & 0x1FFC0000) == 0x18040000
        for ext_id in (0x18040000, 0x18041234, 0x1807FFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"\r",
                             f"EXT ID {ext_id:#x} should pass (incidental match)")

        # EXT IDs outside that range must be blocked
        for ext_id in (0x18030000, 0x18080000, 0x00000000, 0x1FFFFFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT ID {ext_id:#x} should be blocked")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_two_std_ids(self):
        """
        Doc Example 3: typical dual filter use case — two disjoint STD IDs.
        Code=20004000, Mask=001F001F

        Bit calculation:
          Filter 1: AC0=0x20, AC1=0x00  ->  code_std_f1 = (0x20<<3)|(0x00>>5) = 0x100
          Filter 2: AC2=0x40, AC3=0x00  ->  code_std_f2 = (0x40<<3)|(0x00>>5) = 0x200
          AM0=AM2=0x00, AM1=AM3=0x1F  ->  mask = 0x000 (exact match for both filters)

        A single filter cannot express 0x100 OR 0x200; dual mode handles it.
        EXT: incidentally accepts 0x04000000..0x0403FFFF (F1) and
             0x08000000..0x0803FFFF (F2).
        """
        self.dut.send(b"M20004000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m001F001F\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # STD ID 0x100 must pass (Filter 1)
        self.dut.send(b"t1000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t1000\r")

        # STD ID 0x200 must pass (Filter 2)
        self.dut.send(b"t2000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t2000\r")

        # Other STD IDs must be blocked
        for std_id in (0x000, 0x0FF, 0x101, 0x1FF, 0x201, 0x300, 0x7FF):
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD ID {std_id:#x} should be blocked")

        # EXT IDs in Filter 1 incidental range 0x04000000..0x0403FFFF must pass
        #   code_ext_f1 = (0x20<<21)|(0x00<<13) = 0x04000000
        #   mask_ext_f1 = (0x00<<21)|(0x1F<<13)|0x1FFF = 0x0003FFFF
        #   STM32 mask = 0x1FFC0000 -> accepted: (ID & 0x1FFC0000) == 0x04000000
        for ext_id in (0x04000000, 0x04001234, 0x0403FFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"\r",
                             f"EXT ID {ext_id:#x} should pass (Filter 1 incidental)")

        # EXT IDs in Filter 2 incidental range 0x08000000..0x0803FFFF must pass
        #   code_ext_f2 = (0x40<<21)|(0x00<<13) = 0x08000000
        for ext_id in (0x08000000, 0x08001234, 0x0803FFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"\r",
                             f"EXT ID {ext_id:#x} should pass (Filter 2 incidental)")

        # EXT IDs outside both ranges must be blocked
        for ext_id in (0x00000000, 0x03FC0000, 0x04040000, 0x07FC0000,
                       0x08040000, 0x0137FEC8, 0x1FFFFFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT ID {ext_id:#x} should be blocked")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_ext_id_range(self):
        """
        Doc Example 4: EXT ID range 0x18DA0000..0x18DAFFFF (both filters same).
        Code=C6D7C6D7, Mask=00070007

        Bit calculation:
          AC0=0xC6, AC1=0xD7  ->  code_ext_f1 = (0xC6<<21)|(0xD7<<13) = 0x18DAE000
          AM0=0x00, AM1=0x07  ->  mask_ext_f1 = (0x07<<13)|0x1FFF = 0x0000FFFF
          STM32 mask = 0x1FFF0000  ->  accepted: (ID & 0x1FFF0000) == 0x18DA0000

          code_std_f1 = (0xC6<<3)|(0xD7>>5) = 0x630|0x06 = 0x636
          mask_std_f1 = (0x00<<3)|(0x07>>5) = 0x000 (exact)  ->  only STD 0x636 passes.
        """
        self.dut.send(b"MC6D7C6D7\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00070007\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # EXT IDs in range 0x18DA0000..0x18DAFFFF must pass
        for ext_id in (0x18DA0000, 0x18DA1234, 0x18DAFFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"\r",
                             f"EXT ID {ext_id:#x} should pass")

        # EXT IDs outside 0x18DAxxxx must be blocked
        for ext_id in (0x17DA0000, 0x18D90000, 0x18DB0000, 0x00000000, 0x1FFFFFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT ID {ext_id:#x} should be blocked")

        # STD ID 0x636 passes incidentally; adjacent IDs are blocked
        self.dut.send(b"t6360\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t6360\r")
        self.dut.send(b"t6350\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        self.dut.send(b"t6370\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_every_std_id_bit(self):
        """
        Verify that each of the 11 STD CAN ID bits is independently compared
        by the dual filter.
        Both filters are configured to accept only STD ID 0x000 (exact match).
        Code=00000000, Mask=00000000:
          code_std = (0x00<<3)|(0x00>>5) = 0x000
          mask_std = 0x000 -> STM32 mask = 0x7FF (all bits compared)
        """
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # STD ID 0x000 must pass
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t0000\r")

        # Each ID with exactly one bit set must be blocked (tests all 11 bit positions)
        for bit in range(0, 11):
            std_id = 1 << bit
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD ID {std_id:#x} (bit {bit}) should be blocked")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_ext_upper_bits_filtered_lower_dont_care(self):
        """
        In dual filter mode, EXT ID bits 12:0 are always don't-care;
        only bits 28:13 are compared.
        Code=00000000, Mask=00000000:
          code_ext = 0x00000000
          mask_ext = 0x00001FFF (lower 13 bits forced don't-care)
          STM32 mask = 0x1FFFE000 -> accepted: (ID & 0x1FFFE000) == 0x00000000
          i.e., EXT IDs 0x00000000..0x00001FFF pass.
        """
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # EXT IDs 0x00000000..0x00001FFF must pass
        # (lower 13 bits vary freely, upper bits are all 0)
        for ext_id in (0x00000000, 0x00000001, 0x00001000, 0x00001FFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"\r",
                             f"EXT ID {ext_id:#x} should pass (bits 12:0 are don't-care)")

        # Each EXT ID with exactly one bit set in bits 28:13 must be blocked
        for bit in range(13, 29):
            ext_id = 1 << bit
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT ID {ext_id:#x} (bit {bit}) should be blocked")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_mode_switch_w0_w2_w0(self):
        """
        Switching from W0 to W2 and back to W0 correctly re-enables the
        second filter slot with the original code/mask values.

        Config: Code=20004000, Mask=001F001F
          W0: Filter1->0x100, Filter2->0x200 (both STD IDs pass)
          W2: code bit31=0 and mask bit31=0 -> state_std=DISABLE; all STD blocked
          W0 again: second filter re-applied -> 0x100 and 0x200 pass again
        """
        # Configure dual filter: Filter1->0x100, Filter2->0x200
        self.dut.send(b"M20004000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m001F001F\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # --- W0: both 0x100 and 0x200 must pass ---
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t1000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t1000\r")
        self.dut.send(b"t2000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t2000\r")
        self.dut.send(b"t3000\r")
        self.assertEqual(self.dut.receive(), b"z\r")   # third ID is blocked
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # --- Switch to W2 (simple filter mode) ---
        # With code=0x20004000 bit31=0 and mask=0x001F001F bit31=0:
        # gen_configure_filter sets state_std = DISABLE -> all STD IDs blocked
        self.dut.send(b"W2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t1000\r")
        self.assertEqual(self.dut.receive(), b"z\r")   # blocked under W2
        self.dut.send(b"t2000\r")
        self.assertEqual(self.dut.receive(), b"z\r")   # blocked under W2
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # --- Switch back to W0 ---
        # Both filter slots must be re-applied with the stored code/mask
        self.dut.send(b"W0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t1000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t1000\r")   # passes again
        self.dut.send(b"t2000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t2000\r")   # passes again
        self.dut.send(b"t3000\r")
        self.assertEqual(self.dut.receive(), b"z\r")   # still blocked
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


class SimpleFilterTestCase(unittest.TestCase):
    """Test cases for the W2 simple filter mode."""

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()

    def tearDown(self):
        self.dut.close()


    def test_simple_filter_basic(self):
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        #self.dut.print_on = True

        # Check pass all filter (default)
        self.dut.send(b"W2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
            self.dut.send(cmd + b"7C00\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"7C00\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0000003F0\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0000003F0\r")
            self.dut.send(cmd + b"000007C00\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"000007C00\r")
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
            self.dut.send(cmd + b"1EC801370\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"1EC801370\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check pass 0x03F and 0x0000003F filter
        self.dut.send(b"W2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"M0000003F\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"mFFFFF800\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
            self.dut.send(cmd + b"7C00\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0000003F0\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0000003F0\r")
            self.dut.send(cmd + b"000007C00\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"1EC801370\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check pass 0x6C8 and 0x0137FEC8 filter
        self.dut.send(b"M0137FEC8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"mE0000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
            self.dut.send(cmd + b"7C00\r")
            self.assertEqual(self.dut.receive(), b"z\r")
            self.dut.send(cmd + b"6C80\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"6C80\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0000003F0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"000007C00\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
            self.dut.send(cmd + b"1EC801370\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check pass STD ID 0x03F filter
        self.dut.send(b"M8000003F\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
            self.dut.send(cmd + b"7C00\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0000003F0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"000007C00\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"1EC801370\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check pass EXT ID 0x0137FEC8 filter
        self.dut.send(b"M0137FEC8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
            self.dut.send(cmd + b"7C00\r")
            self.assertEqual(self.dut.receive(), b"z\r")
            self.dut.send(cmd + b"6C80\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0000003F0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"000007C00\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
            self.dut.send(cmd + b"1EC801370\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_simple_filter_every_bits(self):
        # Check pass STD ID 0x000 filter comparing every bit in CAN ID
        self.dut.send(b"W2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"M80000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t0000\r")
        for idx in range(0, 11):
            self.dut.send(b"t" + (f'{(1 << idx):03X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        self.dut.send(b"T000000000\r")
        self.assertEqual(self.dut.receive(), b"Z\r")
        for idx in range(0, 29):
            self.dut.send(b"T" + (f'{(1 << idx):08X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check pass EXT ID 0x00000000 filter comparing every bit in CAN ID
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        for idx in range(0, 11):
            self.dut.send(b"t" + (f'{(1 << idx):03X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        self.dut.send(b"T000000000\r")
        self.assertEqual(self.dut.receive(), b"Z\r" + b"T000000000\r")
        for idx in range(0, 29):
            self.dut.send(b"T" + (f'{(1 << idx):08X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check pass 0x000 and 0x00000000 filter comparing every bit in CAN ID
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m80000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t0000\r")
        for idx in range(0, 11):
            self.dut.send(b"t" + (f'{(1 << idx):03X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        self.dut.send(b"T000000000\r")
        self.assertEqual(self.dut.receive(), b"Z\r" + b"T000000000\r")
        for idx in range(0, 29):
            self.dut.send(b"T" + (f'{(1 << idx):08X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check consistency by setting pass STD ID 0x000 filter again
        self.dut.send(b"m80000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t0000\r")
        for idx in range(0, 11):
            self.dut.send(b"t" + (f'{(1 << idx):03X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        self.dut.send(b"T000000000\r")
        self.assertEqual(self.dut.receive(), b"Z\r" + b"T000000000\r")
        for idx in range(0, 29):
            self.dut.send(b"T" + (f'{(1 << idx):08X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
