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
        # All tests below run under W2; switch once here. setUp's setup()
        # has already left Code/Mask at defaults (M00000000, mFFFFFFFF).
        self.dut.send(b"W2\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def tearDown(self):
        self.dut.close()


    def test_pass_all(self):
        """Doc Example 1: W2 with mFFFFFFFF (all bits don't-care) passes
        all standard and extended frames. This is the default power-on
        state."""
        # setUp leaves Code=00000000, Mask=FFFFFFFF.
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Standard frames: a representative sample across the ID range.
        for std_id in (0x000, 0x001, 0x100, 0x3FF, 0x601, 0x7FE, 0x7FF):
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"\r",
                             f"STD ID {std_id:#x} should pass with mFFFFFFFF")

        # Extended frames: a representative sample across the ID range.
        for ext_id in (0x00000000, 0x00000001, 0x0137FEC8, 0x18DA0000, 0x1FFFFFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"\r",
                             f"EXT ID {ext_id:#x} should pass with mFFFFFFFF")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_std_id_only(self):
        """Doc Example 2: Code=80000100, Mask=00000000 accepts only STD ID 0x100.

        Bit calculation:
          M[31]=1, m[31]=0 -> ~IDE=1 required -> STD frames only
          M[10:0]=0x100, m[10:0]=0x000 -> exact match: STD ID 0x100
          (M[28:11] are not mapped to a STD ID; with m=0 they are compared
          against the implicit 0 of the STD frame's upper bits, but the
          frame type itself is already constrained by M[31].)
        """
        self.dut.send(b"M80000100\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # STD 0x100 must pass.
        self.dut.send(b"t1000\r")
        self.assertEqual(self.dut.receive(), b"z\rt1000\r",
                         "STD ID 0x100 should pass")

        # Adjacent and other STD IDs must be blocked.
        for std_id in (0x000, 0x0FF, 0x101, 0x1FF, 0x200, 0x7FF):
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD ID {std_id:#x} should be blocked")

        # All EXT IDs must be blocked (frame type mismatch).
        for ext_id in (0x00000000, 0x00000100, 0x0137FEC8, 0x1FFFFFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT ID {ext_id:#x} should be blocked (STD-only filter)")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_ext_id_range(self):
        """Doc Example 3: Code=18DB0000, Mask=0000FFFF accepts EXT ID range
        0x18DB0000 - 0x18DBFFFF.

        Bit calculation:
          M[31]=0, m[31]=0 -> ~IDE=0 required -> EXT frames only
          M[28:16]=0x18DB, m[28:16]=0x0000 -> exact match for upper 13 ID bits
          M[15:0]=0x0000, m[15:0]=0xFFFF -> lower 16 ID bits don't-care
          -> accepted: (ID & 0x1FFF0000) == 0x18DB0000
        """
        self.dut.send(b"M18DB0000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m0000FFFF\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # EXT IDs in the range 0x18DB0000..0x18DBFFFF must pass.
        for ext_id in (0x18DB0000, 0x18DB0001, 0x18DB1234, 0x18DBFFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"\r",
                             f"EXT ID {ext_id:#x} should pass")

        # EXT IDs outside that range must be blocked.
        for ext_id in (0x00000000, 0x18DA0000, 0x18DC0000, 0x1FFFFFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT ID {ext_id:#x} should be blocked")

        # All STD IDs must be blocked (frame type mismatch).
        for std_id in (0x000, 0x100, 0x7FF):
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD ID {std_id:#x} should be blocked (EXT-only filter)")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_std_and_ext_with_dont_care_ide(self):
        """Doc Example 4: Code=00000100, Mask=80000000 accepts STD ID 0x100
        AND EXT ID 0x00000100. The leading 8 in mask marks the ~IDE bit
        as don't-care, so the same low ID matches both frame types.

        Bit calculation:
          M[31]=0, m[31]=1 -> ~IDE don't-care -> both STD and EXT accepted
          M[28:0]=0x00000100, m[28:0]=0x00000000 -> exact match on 29 bits
          STD interpretation: M[10:0]=0x100 -> STD ID 0x100
          EXT interpretation: M[28:0]=0x00000100 -> EXT ID 0x00000100
        """
        self.dut.send(b"M00000100\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m80000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # STD 0x100 must pass.
        self.dut.send(b"t1000\r")
        self.assertEqual(self.dut.receive(), b"z\rt1000\r",
                         "STD ID 0x100 should pass")
        # EXT 0x00000100 must pass.
        self.dut.send(b"T000001000\r")
        self.assertEqual(self.dut.receive(), b"Z\rT000001000\r",
                         "EXT ID 0x00000100 should pass")

        # Other STD IDs must be blocked.
        for std_id in (0x000, 0x101, 0x7FF):
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD ID {std_id:#x} should be blocked")

        # Other EXT IDs must be blocked.
        for ext_id in (0x00000000, 0x00000101, 0x0137FEC8, 0x1FFFFFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT ID {ext_id:#x} should be blocked")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_ext_id_only(self):
        """EXT-only counterpart to Example 2: Code=0137FEC8, Mask=00000000
        accepts only EXT ID 0x0137FEC8.

        Bit calculation:
          M[31]=0, m[31]=0 -> ~IDE=0 required -> EXT frames only
          M[28:0]=0x0137FEC8, m=0 -> exact match: EXT ID 0x0137FEC8
        """
        self.dut.send(b"M0137FEC8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # EXT 0x0137FEC8 must pass.
        self.dut.send(b"T0137FEC80\r")
        self.assertEqual(self.dut.receive(), b"Z\rT0137FEC80\r",
                         "EXT ID 0x0137FEC8 should pass")

        # Other EXT IDs must be blocked.
        for ext_id in (0x00000000, 0x0137FEC7, 0x0137FEC9, 0x1EC80137, 0x1FFFFFFF):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT ID {ext_id:#x} should be blocked")

        # All STD IDs must be blocked (frame type mismatch).
        for std_id in (0x000, 0x6C8, 0x7FF):   # 0x6C8 = 0x0137FEC8 & 0x7FF
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD ID {std_id:#x} should be blocked (EXT-only filter)")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_lower_11bit_only_with_dont_care_ide(self):
        """Code=0000003F, Mask=FFFFF800: only the lower 11 ID bits are
        compared and the frame type is don't-care. Accepts STD ID 0x03F
        and EXT IDs whose lower 11 bits are 0x03F.

        Bit calculation:
          M[31]=0, m[31]=1 -> ~IDE don't-care
          M[10:0]=0x03F, m[10:0]=0x000 -> exact match on lower 11 bits
          M[28:11]=0, m[28:11]=all 1 -> upper 18 bits don't-care
        """
        self.dut.send(b"M0000003F\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"mFFFFF800\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # STD 0x03F must pass.
        self.dut.send(b"t03F0\r")
        self.assertEqual(self.dut.receive(), b"z\rt03F0\r",
                         "STD ID 0x03F should pass")

        # Other STD IDs must be blocked.
        for std_id in (0x000, 0x03E, 0x040, 0x7FF):
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD ID {std_id:#x} should be blocked")

        # EXT IDs with lower 11 bits == 0x03F must pass, regardless of upper bits.
        for ext_id in (0x0000003F, 0x0000083F, 0x0001003F, 0x1FFFF83F):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"\r",
                             f"EXT ID {ext_id:#x} (lower 11 bits = 0x03F) should pass")

        # EXT IDs with different lower 11 bits must be blocked.
        for ext_id in (0x00000000, 0x0000003E, 0x0000043F, 0x0137FEC8):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT ID {ext_id:#x} should be blocked")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_same_code_distinct_std_and_ext_ids(self):
        """Code=0137FEC8, Mask=E0000000 demonstrates that the same code
        value is interpreted as different IDs for STD and EXT frames:
        STD takes only the lower 11 bits, EXT takes 29 bits.

        Bit calculation:
          M[31]=0, m[31]=1 -> ~IDE don't-care
          m[30:29]=0b11 -> upper 2 bits don't-care (they are AC0[6:5] and
            are not part of the 29-bit EXT ID anyway, so the effect is nil
            for this code value; included to mirror the original test)
          M[28:0]=0x0137FEC8, m[28:0]=0 -> exact match on the 29-bit code
          STD interpretation: M[10:0]=0x6C8 (= 0x0137FEC8 & 0x7FF)
          EXT interpretation: M[28:0]=0x0137FEC8
        """
        self.dut.send(b"M0137FEC8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"mE0000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # STD 0x6C8 must pass (the lower 11 bits of M).
        self.dut.send(b"t6C80\r")
        self.assertEqual(self.dut.receive(), b"z\rt6C80\r",
                         "STD ID 0x6C8 should pass")
        # EXT 0x0137FEC8 must pass (the full 29 bits of M).
        self.dut.send(b"T0137FEC80\r")
        self.assertEqual(self.dut.receive(), b"Z\rT0137FEC80\r",
                         "EXT ID 0x0137FEC8 should pass")

        # Other STD IDs must be blocked.
        for std_id in (0x000, 0x03F, 0x6C7, 0x6C9, 0x7FF):
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD ID {std_id:#x} should be blocked")

        # Other EXT IDs must be blocked.
        for ext_id in (0x00000000, 0x0000003F, 0x0137FEC7, 0x1EC80137):
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT ID {ext_id:#x} should be blocked")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_every_std_id_bit(self):
        """Verify each of the 11 STD CAN ID bits is independently compared.
        Code=80000000, Mask=00000000 accepts only STD ID 0x000.

        Bit calculation:
          M[31]=1, m[31]=0 -> STD frames only
          M[10:0]=0x000, m[10:0]=0 -> exact match: STD ID 0x000
        """
        self.dut.send(b"M80000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # STD 0x000 must pass.
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\rt0000\r",
                         "STD ID 0x000 should pass")

        # Each STD ID with exactly one bit set must be blocked.
        for bit in range(0, 11):
            std_id = 1 << bit
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD ID {std_id:#x} (bit {bit}) should be blocked")

        # All EXT IDs must be blocked (frame type mismatch).
        self.dut.send(b"T000000000\r")
        self.assertEqual(self.dut.receive(), b"Z\r",
                         "EXT ID 0x00000000 should be blocked (STD-only filter)")
        for bit in range(0, 29):
            ext_id = 1 << bit
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT ID {ext_id:#x} (bit {bit}) should be blocked (STD-only filter)")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_every_ext_id_bit(self):
        """Verify each of the 29 EXT CAN ID bits is independently compared.
        Code=00000000, Mask=00000000 accepts only EXT ID 0x00000000.

        Bit calculation:
          M[31]=0, m[31]=0 -> EXT frames only
          M[28:0]=0, m[28:0]=0 -> exact match: EXT ID 0x00000000
        """
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # EXT 0x00000000 must pass.
        self.dut.send(b"T000000000\r")
        self.assertEqual(self.dut.receive(), b"Z\rT000000000\r",
                         "EXT ID 0x00000000 should pass")

        # Each EXT ID with exactly one bit set must be blocked.
        for bit in range(0, 29):
            ext_id = 1 << bit
            cmd = ("T" + f"{ext_id:08X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT ID {ext_id:#x} (bit {bit}) should be blocked")

        # All STD IDs must be blocked (frame type mismatch).
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r",
                         "STD ID 0x000 should be blocked (EXT-only filter)")
        for bit in range(0, 11):
            std_id = 1 << bit
            cmd = ("t" + f"{std_id:03X}" + "0").encode()
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD ID {std_id:#x} (bit {bit}) should be blocked (EXT-only filter)")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_command_order_independence(self):
        """The order in which M and m are sent must not affect the resulting
        filter state. Configure Code=00000000, Mask=80000000 (STD 0x000 +
        EXT 0x00000000) in both orders and confirm the same pass/block set."""
        # --- Order 1: M then m ---
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m80000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\rt0000\r",
                         "Order M->m: STD 0x000 should pass")
        self.dut.send(b"T000000000\r")
        self.assertEqual(self.dut.receive(), b"Z\rT000000000\r",
                         "Order M->m: EXT 0x00000000 should pass")
        self.dut.send(b"t1000\r")
        self.assertEqual(self.dut.receive(), b"z\r",
                         "Order M->m: STD 0x100 should be blocked")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # --- Order 2: m then M (swapped) ---
        self.dut.send(b"m80000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\rt0000\r",
                         "Order m->M: STD 0x000 should pass (same result as M->m)")
        self.dut.send(b"T000000000\r")
        self.assertEqual(self.dut.receive(), b"Z\rT000000000\r",
                         "Order m->M: EXT 0x00000000 should pass (same result as M->m)")
        self.dut.send(b"t1000\r")
        self.assertEqual(self.dut.receive(), b"z\r",
                         "Order m->M: STD 0x100 should be blocked (same result as M->m)")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_mode_switch_w2_w0_w2(self):
        """Switching from W2 to W0 and back to W2 correctly re-applies the
        simple-mode filter. The Code/Mask values are preserved across mode
        changes; only the interpretation is switched.

        Config: Code=80000100, Mask=00000000
          W2: STD ID 0x100 only (Doc Example 2)
          W0: same Code/Mask reinterpreted as dual filter -> different pass set
          W2 again: STD ID 0x100 again
        """
        self.dut.send(b"M80000100\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # --- W2: STD 0x100 must pass; STD 0x000 blocked ---
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t1000\r")
        self.assertEqual(self.dut.receive(), b"z\rt1000\r",
                         "W2: STD 0x100 should pass")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r",
                         "W2: STD 0x000 should be blocked")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # --- W0: same Code/Mask now interpreted as dual filter ---
        # Under W0 the same code 0x80000100 yields different filter values
        # (the bit-mapping switches). STD 0x100 is not expected to pass any
        # more; the exact dual-mode behaviour is covered by DualFilterTestCase.
        self.dut.send(b"W0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t1000\r")
        self.assertEqual(self.dut.receive(), b"z\r",
                         "W0: STD 0x100 should NOT pass (code/mask reinterpreted as dual filter)")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # --- W2 again: original behaviour must be restored ---
        self.dut.send(b"W2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t1000\r")
        self.assertEqual(self.dut.receive(), b"z\rt1000\r",
                         "W2 restored: STD 0x100 should pass again")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r",
                         "W2 restored: STD 0x000 should still be blocked")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
