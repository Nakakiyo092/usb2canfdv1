#!/usr/bin/env python3

"""
A Python script that demonstrates basic CAN-FD send and receive against an
slcan-compatible device via python-can's slcan interface.

The script runs in one of two roles:
- tx: transmits one classic (`t`), one FD without BRS (`d`) and one FD with
      BRS (`b`) frame to a fixed CAN ID once per second, for 10 seconds.
- rx: receives every frame on the bus indefinitely and prints it. Start
      this role first when running two instances on two devices.

Bitrate is configured through python-can's BitTimingFd path. Only the
nominal and data bitrates inside the BitTimingFd object are honoured by
the slcan backend; the f_clock, segment values and sample points are
ignored (the constructor still requires them, so this script passes
plausible dummies and labels them as such). An optional --nom-custom
overrides nominal afterwards via set_bitrate_reg, which sends the
slcan lowercase `s` command without disturbing the data bitrate.

Usage:
    Connect an slcan-compatible device, then run from the repository root:

    Windows (PowerShell):
        python test/example_txrx.py <devicename> --role {tx,rx} [options]

    Linux or macOS:
        python3 test/example_txrx.py <devicename> --role {tx,rx} [options]

    Arguments:
        devicename    CAN device name (e.g., COM9 on Windows, /dev/ttyACM0 on Linux)

    Options:
        -h, --help                  Show this help message and exit
        --role {tx,rx}              Required. tx = periodic send for 10 s,
                                    rx = receive and print until Ctrl+C
        -S, --nom KBPS              Nominal bitrate in kbps. Choices: 125, 250,
                                    500, 1000 (default: 500)
        -Y, --data KBPS             Data bitrate in kbps. Choices: 2000, 5000
                                    (default: 2000)
        -s, --nom-custom HEX        Nominal custom timing as an `s` command
                                    body (4- or 8-hex). Overrides --nom.

    Examples:
        python test/example_txrx.py COM8 --role rx
        python test/example_txrx.py COM9 --role tx
        python test/example_txrx.py COM9 --role tx -S 1000 -Y 5000
        python test/example_txrx.py COM9 --role tx -s 013F1010
"""

import argparse
import time

import can
from can import BitTimingFd

# Fixed demo parameters.
_TX_CAN_ID = 0x123
_TX_DURATION_S = 10
_TX_INTERVAL_S = 1.0
_TX_DATA = bytes(range(1, 9))  # 01 02 03 04 05 06 07 08

# Choices limited to bitrates reliably supported via python-can's slcan API.
_NOM_BITRATE_CHOICES_KBPS = (125, 250, 500, 1000)
_DATA_BITRATE_CHOICES_KBPS = (2000, 5000)

# Dummy CAN system clock for BitTimingFd construction (ignored by the slcan
# backend; only nom_bitrate and data_bitrate inside BitTimingFd are honoured).
_DUMMY_F_CLOCK_HZ = 80_000_000


def _get_argparser():
    """Get the command line argument parser."""
    parser = argparse.ArgumentParser(
        description="CAN-FD tx/rx example for an slcan device. Press Ctrl+C to quit."
    )
    parser.add_argument(
        "devicename",
        type=str,
        help="device name like COM9 or /dev/ttyACM0 (required)",
    )
    parser.add_argument(
        "--role",
        choices=("tx", "rx"),
        required=True,
        help="tx = transmit t/d/b frames periodically; rx = listen and print",
    )
    parser.add_argument(
        "-S", "--nom",
        type=int,
        choices=_NOM_BITRATE_CHOICES_KBPS,
        default=500,
        metavar="KBPS",
        help=f"nominal bitrate in kbps {_NOM_BITRATE_CHOICES_KBPS} (default: 500)",
    )
    parser.add_argument(
        "-Y", "--data",
        type=int,
        choices=_DATA_BITRATE_CHOICES_KBPS,
        default=2000,
        metavar="KBPS",
        help=f"data bitrate in kbps {_DATA_BITRATE_CHOICES_KBPS} (default: 2000)",
    )
    parser.add_argument(
        "-s", "--nom-custom",
        type=str,
        default=None,
        metavar="HEX",
        help="nominal custom timing as the body of the slcan `s` command "
             "(e.g. '013F1010' for the 8-hex prescaler/tseg1/tseg2/sjw form, "
             "or the 4-hex SJA1000 BTR0/BTR1 form); overrides --nom",
    )
    return parser


def _create_bus(args):
    """Create and return the CAN bus, or None if initialization fails."""
    # BitTimingFd is the canonical python-can API for FD bitrate setup, but
    # the slcan backend only reads .nom_bitrate and .data_bitrate from it.
    # f_clock, the segment values and the sample points are all ignored
    # downstream. They are passed here because the constructor requires
    # them; the values are plausible dummies, not authoritative.
    timing = BitTimingFd.from_sample_point(
        f_clock=_DUMMY_F_CLOCK_HZ,  # ignored by slcan
        nom_bitrate=args.nom * 1000,
        nom_sample_point=87.5,      # ignored by slcan (dummy)
        data_bitrate=args.data * 1000,
        data_sample_point=77.5,     # ignored by slcan (dummy)
    )

    try:
        if args.devicename == "virtual":
            # No bitrate / timing arguments for the virtual interface; it
            # passes Message objects through without modelling the bus.
            return can.Bus("test", interface="virtual")
        bus = can.Bus(
            interface="slcan",
            channel=args.devicename,
            timing=timing,
        )
    except can.CanInitializationError as err:
        print("Could not access CAN network.")
        print("The program is aborting.")
        print(err)
        if args.devicename != "virtual":
            print("Possible causes:")
            print(f"  - Wrong device name: check '{args.devicename}' is correct")
            print( "  - Device not powered: check the device is powered on")
            print( "  - Device not connected: check the device is properly connected")
            print( "  - Wrong firmware: check the device has correct firmware")
            print( "  - Permission denied (Linux): try 'sudo usermod -aG dialout $USER' and re-login")
            print(f"    or 'sudo chmod 666 {args.devicename}'")
        return None
    except Exception as err:
        print("Could not access CAN network.")
        print("The program is aborting.")
        print(err)
        return None

    # Optional nominal-only override via the slcan lowercase `s` command.
    # set_bitrate_reg writes "s" + the given hex string and reopens the bus;
    # the data bitrate established by the BitTimingFd above is preserved
    # because the `s` command only updates the nominal config.
    if args.nom_custom:
        try:
            bus.set_bitrate_reg(args.nom_custom)
        except Exception as err:
            print(f"Could not apply nominal custom timing '{args.nom_custom}'.")
            print(err)
            bus.shutdown()
            return None

    return bus


def _run_tx(bus):
    """Send one t -> d -> b cycle per second for the configured duration.

    Returns the number of frames the slcan bus accepted (send did not raise).
    """
    print(f"TX: sending t/d/b frames to ID 0x{_TX_CAN_ID:X} for {_TX_DURATION_S}s")

    # Frame templates: classic, FD without BRS, FD with BRS.
    templates = [
        ("t", can.Message(arbitration_id=_TX_CAN_ID, data=_TX_DATA,
                          is_extended_id=False, is_fd=False)),
        ("d", can.Message(arbitration_id=_TX_CAN_ID, data=_TX_DATA,
                          is_extended_id=False, is_fd=True, bitrate_switch=False)),
        ("b", can.Message(arbitration_id=_TX_CAN_ID, data=_TX_DATA,
                          is_extended_id=False, is_fd=True, bitrate_switch=True)),
    ]

    deadline = time.monotonic() + _TX_DURATION_S
    sent = 0
    i = 0
    while time.monotonic() < deadline:
        label, msg = templates[i % len(templates)]
        try:
            bus.send(msg)
            sent += 1
            print(f"TX [{label}] ID=0x{_TX_CAN_ID:X} data={msg.data.hex().upper()}")
        except can.CanError as err:
            print(f"TX [{label}] failed: {err}")
        i += 1
        time.sleep(_TX_INTERVAL_S)
    print(f"TX: done. Sent {sent} frames.")
    return sent


def _run_rx(bus):
    """Listen for frames until Ctrl+C and print each one.

    Uses python-can's standard Message.__str__ format so the output looks
    the same as can.Printer / can.Logger and other tools in the ecosystem.

    Returns the number of frames received.
    """
    print("RX: listening (Ctrl+C to stop)")
    received = 0
    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue
            print(msg)
            received += 1
    except KeyboardInterrupt:
        print()  # newline after ^C
    print(f"RX: done. Received {received} frames.")
    return received


def _query_f(bus, timeout=1.0):
    """Send the slcan `F` status command and return the 2-hex response.

    Returns the status hex string (e.g. '00', 'A4') or None on timeout.
    Uses the slcan backend's private _write / _read so we can interleave
    a status request with frame traffic on the same bus -- python-can
    does not expose a public method for the `F` command.
    Any non-`F` strings read while we wait are stashed back into the
    backend's queue so they are not lost.
    """
    bus._write("F")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        s = bus._read(0.1)
        if not s:
            continue
        if s[0] == "F" and len(s) >= 3:
            return s[1:-1]  # strip leading 'F' and trailing '\r'
        bus._queue.put_nowait(s)
    return None


def main():
    """Main process."""
    args = _get_argparser().parse_args()

    bus = _create_bus(args)
    if bus is None:
        return

    try:
        if args.role == "tx":
            _run_tx(bus)
        else:
            _run_rx(bus)
    finally:
        # Always query F before shutdown so the run can be categorised
        # (bus normal / data loss / bus error). The slcan `F` command
        # clears the flags on read, so this also leaves the device in a
        # clean state for the next run.
        f = _query_f(bus)
        if f is None:
            print("Status (F): no reply (timeout)")
        else:
            print(f"Status (F): F{f}")
        bus.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
