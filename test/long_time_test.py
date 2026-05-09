#!/usr/bin/env python3

"""
Collection of tests which take long time to complete.

- CAN bus error and buffer error rate (loopback as default or with receiver option)
- Verify the us timestamp against the rounding compensation result (~66ms)
- Compare clock accuracy between host and device

License:
    MIT License.
    See the accompanying LICENSE file for full terms.
"""

import time
import random

import argparse
import serial

ROUND_TRIP_TIME_SAMPLES = 10
STATS_INTERVAL_MS = 60_000
TIMESTAMP_PERIOD_US = 3600_000_000
TIMESTAMP_DIFF_THRESHOLD_US = 0xFFFF // 2

def get_argparser():
    """Get argument parser for this script."""
    parser = argparse.ArgumentParser(
        description="Run collection of long time tests. Press [CTRL] + 'c' to quit."
    )
    parser.add_argument(
        "devicename",
        type=str,
        help="device name like COM9 or /dev/ttyACM0 (required)"
    )
    parser.add_argument(
        "-d", "--duration",
        type=int,
        default=1,
        help="time to test in hours"
    )
    parser.add_argument(
        "-w", "--with-receiver",
        action="store_true",
        help="run the test with another device receiving the messages"
    )
    return parser


def setup_device_under_test(dev: serial.Serial, with_receiver: bool):
    """Setup device and print test information."""
    print("usb port name:", dev.port)
    print("")

    dev.write(b"\a\r\r")    # Flush the buffer
    dev.write(b"C\r")
    time.sleep(0.1)
    dev.read_all()

    dev.write(b"N\r")
    time.sleep(0.1)
    print("serial number:", dev.read_all().decode())
    dev.write(b"V\r")
    time.sleep(0.1)
    print("slcan version:", dev.read_all().decode())
    dev.write(b"v\r")
    time.sleep(0.1)
    print("detail:")
    print("   ", dev.read_all().decode())
    print("")

    # Setup maximum CAN speed to stress the device
    dev.write(b"S8\r")
    dev.write(b"Y5\r")
    dev.write(b"z2002\r")
    time.sleep(0.1)
    dev.read_all()
    
    if with_receiver:
        dev.write(b"O\r")    # TODO: warning if loopback is not supported
        time.sleep(0.1)
        dev.read_all()
        print("can port status: open/normal (1M/5Mbps)")
    else:
        dev.write(b"+\r")
        time.sleep(0.1)
        dev.read_all()
        print("can port status: open/loopback (1M/5Mbps)")

    print("")


def cleanup_device_under_test(dev: serial.Serial):
    """Close the device cleanly at the end of the test."""
    dev.write(b"C\r")
    time.sleep(0.1)
    dev.read_all()
    dev.close()


def print_round_trip_time(dev: serial.Serial) -> int:
    """Print round-trip time. Return average RTT in us.
    
    Returns -1 on error.
    """
    rtt = []
    for _ in range(0, ROUND_TRIP_TIME_SAMPLES):
        # Use perf_counter for better resolution (RTT is expected to be less than ms)
        time_start = time.perf_counter()
        dev.write(b"\r")
        dev.read_until(b"\r")
        time_end = time.perf_counter()
        rtt.append(int((time_end - time_start) * 1000 * 1000))

    print("ping:", rtt, "us")
    if 6 * sum(rtt) // len(rtt) > TIMESTAMP_DIFF_THRESHOLD_US:  # Six sigma
        print("WARNING: The round trip time is too large to verify the timestamp.")
    print("")

    if len(rtt) == 0:
        return -1

    return sum(rtt) // len(rtt)


def make_data_to_write() -> bytes:
    """Make data to write to device."""
    single_msg = b"00112233445566778899AABBCCDDEEFF"
    single_msg = b"B00000000F" + single_msg * 4 + b"\r"

    data_write = single_msg

    return data_write


def extract_timestamp_from_tx_event(msg: bytes) -> int:
    """Extract 4-byte microsecond timestamp from TX event message.
    
    TX Event format: b'Z' + frame_data + timestamp_hex(8 chars)
    Returns timestamp in microseconds (0-3,600,000,000).
    Returns -1 on error.
    """
    if len(msg) < 10 or msg[0:1] != b"Z":
        return -1
    
    try:
        timestamp_hex = msg[-9:-1].decode()     # Last 8 chars before '\r'
        return int(timestamp_hex, 16)
    except (ValueError, UnicodeDecodeError):
        print("ERROR: Failed to extract timestamp from message:", msg)
        return -1


def calc_timestamp_diff(ts_new: int, ts_old: int) -> int:
    """Calculate timestamp difference with 66ms period compensation.
    
    Timestamp counter resets at 3600_000_000 us.
    Returns difference in microseconds.
    """
    if ts_new >= ts_old:
        return ts_new - ts_old
    else:
        # Overflow occurred
        return (TIMESTAMP_PERIOD_US - ts_old) + ts_new


def print_status_check(stats: dict):
    """Print status check results."""
    print(f"status check: {stats['st_checked']} samples")
    print(f"  no error: {stats['st_no_error_count']}")
    print(f"  buffer error: {stats['st_buf_error_count']}")
    print(f"  can bus error: {stats['st_can_error_count']}")
    if stats['st_buf_error_count'] or stats['st_can_error_count']:
        print("WARNING: There was buffer or bus error which can affect timer verification.")
    print("")


def print_timestamp_verification(stats: dict):
    """Print timestamp verification results."""
    if stats["ts_verified"] == 0:
        print("timestamp verification: 0 samples (need at least 1)")
        print("")
        return
    
    avg_error = stats["ts_error_sum"] / stats["ts_verified"]
    max_error = stats["ts_error_max"]
    failure_count = stats["ts_failure_count"]
    
    print(f"timestamp verification: {stats['ts_verified']} samples")
    print(f"  average error: {avg_error:.1f} us")
    print(f"  max error: {max_error} us")
    print(f"  failures (>{TIMESTAMP_DIFF_THRESHOLD_US} us): {failure_count}")
    print("")


def print_clock_accuracy(stats: dict):
    """Print clock drift statistics."""
    if stats["clock_samples"] == 0:
        print("clock accuracy: 0 samples (need at least 1)")
        print("")
        return
    print(f"clock accuracy: {stats['clock_duration'] // 1000_000} sec")
    print(f"  clock offset: {stats['clock_drift'] / 1000:.3f} ms")
    if stats['clock_duration'] > 0:
        print(f"  drift upper bound: {stats['clock_drift_upper_bound'] / stats['clock_duration'] * 1000_000:.3f} ppm")
        print(f"  drift lower bound: {stats['clock_drift_lower_bound'] / stats['clock_duration'] * 1000_000:.3f} ppm")
    else:
        print(f"  drift upper bound: N/A ppm")
        print(f"  drift lower bound: N/A ppm")
    print("")


def main():
    """Main function."""
    argparser = get_argparser()
    args = argparser.parse_args()

    try:
        device = serial.Serial(args.devicename, timeout=1, write_timeout=1)
    except Exception as err:
        print("ERROR: Could not open device ", args.devicename)
        print("")
        print(err)
        print("")
        print("The script is aborting.")
        return

    setup_device_under_test(device, args.with_receiver)
    rtt = print_round_trip_time(device)

    data_write = make_data_to_write()

    stats = {
        "tx_requests": 0,
        "tx_complete": 0,
        "st_checked": 0,
        "st_no_error_count": 0,
        "st_buf_error_count": 0,
        "st_can_error_count": 0,
        "st_can_error_count": 0,
        "ts_verified": 0,
        "ts_error_sum": 0,
        "ts_error_max": 0,
        "ts_failure_count": 0,
        "clock_samples": 0,
        "clock_drift": 0,
        "clock_duration": 0,
        "clock_drift_upper_bound": 0,
        "clock_drift_lower_bound": 0,
    }

    # Timestamp tracking
    host_tx_time_us_list = []
    host_tx_time_us_initial = -1
    host_tx_time_us_prev = -1
    device_ts = -1
    device_ts_initial = -1
    device_ts_prev = -1
    pending_rx = b""

    tick_start = int(round(time.time() * 1000))
    tick_tx = tick_start
    tick_stats = tick_start + STATS_INTERVAL_MS
    tick_end = tick_start + 1000 * 3600 * args.duration

    while True:
        # Read and process incoming message.
        chunk = device.read_all()
        if chunk:
            pending_rx += chunk
            while True:
                while pending_rx.startswith(b"\a"):
                    pending_rx = pending_rx[1:]

                delimiter_index = pending_rx.find(b"\r")
                if delimiter_index == -1:
                    break

                msg = pending_rx[:delimiter_index + 1]
                pending_rx = pending_rx[delimiter_index + 1:]

                if msg.startswith(b"F"):
                    stats["st_checked"] += 1
                    if len(msg) >= 3:
                        flags = int(msg[1:3].decode(), 16)
                        if flags & 0b00001011:
                            #print("WARNING: Buffer error detected in status check message:", msg.strip())
                            stats["st_buf_error_count"] += 1
                        elif flags & 0b10110100:
                            #print("WARNING: CAN bus error detected in status check message:", msg.strip())
                            stats["st_can_error_count"] += 1
                        else:
                            stats["st_no_error_count"] += 1
                    else:
                        print("WARNING: Malformed status check message (too short):", msg.strip())

                if msg.startswith(b"Z"):
                    stats["tx_complete"] += 1
                    device_ts = extract_timestamp_from_tx_event(msg)
                    if device_ts < 0:
                        print("The script is aborting.")
                        return

                    # Perform timestamp verification if we have previous values
                    if host_tx_time_us_prev >= 0 and device_ts_prev >= 0 and host_tx_time_us_list:
                        # Compare with the last timestamp
                        host_diff_us = host_tx_time_us_list[0] - host_tx_time_us_prev
                        device_diff_us = calc_timestamp_diff(device_ts, device_ts_prev)
                        error_us = abs(host_diff_us - device_diff_us)
                        
                        stats["ts_verified"] += 1
                        stats["ts_error_sum"] += error_us
                        stats["ts_error_max"] = max(stats["ts_error_max"], error_us)
                        
                        if error_us > TIMESTAMP_DIFF_THRESHOLD_US:
                            print(f"WARNING: Timestamp verification failed for message {msg.strip()}: device_ts={device_ts}, device_ts_prev={device_ts_prev}")
                            stats["ts_failure_count"] += 1

                        # Compare with the initial timestamp
                        host_interval_us = host_tx_time_us_list[0] - host_tx_time_us_initial
                        device_interval_us = calc_timestamp_diff(device_ts, device_ts_initial)
                        drift_us = device_interval_us - host_interval_us
                        while drift_us < -3600_000_000 // 2:
                            drift_us += 3600_000_000
                        # Supposed initial RTT is 0 and current RTT is 2 * ave. RTT (or reverse) as the worst case.
                        # Then include safety margin (six sigma) as the RTT can sometimes be much higher than the average (like x10).
                        drift_upper_bound_us = abs(drift_us) + 6 * 2 * rtt
                        drift_lower_bound_us = abs(drift_us) - 6 * 2 * rtt
                        if drift_lower_bound_us < 0:
                            drift_lower_bound_us = 0

                        stats["clock_samples"] += 1
                        stats["clock_drift"] = drift_us
                        stats["clock_duration"] = host_interval_us
                        stats["clock_drift_upper_bound"] = drift_upper_bound_us
                        stats["clock_drift_lower_bound"] = drift_lower_bound_us

                    # Store initial timestamp if we have the first values
                    elif host_tx_time_us_list:
                        host_tx_time_us_initial = host_tx_time_us_list[0]
                        device_ts_initial = device_ts

                    else:
                        print("ERROR: Something went wrong.")
                        print("The script is aborting.")
                        return

                    host_tx_time_us_prev = host_tx_time_us_list.pop(0)
                    device_ts_prev = device_ts

        ms = int(round(time.time() * 1000))
        if ms > tick_tx:
            rnd = random.randint(0, 100)
            if rnd <= 90:
                # Short delay to check max 2 compensation as a most likely case (66ms * 2 = 132ms)
                tick_tx = ms + random.randint(0, 150)
            elif rnd <= 95:
                # Long delay to check max ~100 compensation as an extreme case (66ms * 100 = 6600ms)
                # The rough device clock accuracy (0.5%) limits the max duration to around 66ms / 2 / 0.005 = 6600ms.
                tick_tx = ms + random.randint(0, 6600)
            else:
                # No delay to stress the buffer and increase the number of frames as another extreme case
                tick_tx = ms + 0

            # Record host TX timestamp in microseconds (perf_counter returns seconds, convert to us)
            host_tx_time_us_list.append(int(round(time.perf_counter() * 1000 * 1000)))

            device.write(b"F\r")    # Status check
            device.write(data_write)
            stats["tx_requests"] += 1

        if ms > tick_stats:
            tick_stats = ms + STATS_INTERVAL_MS

            print("")
            print(f"--- Stats at {(ms - tick_start) / 3600 / 1000:.3f} hours ---")
            print("")
            print(f"sent frames: {stats["tx_complete"]} / {stats["tx_requests"]}")
            print("")
            print_status_check(stats)
            print_timestamp_verification(stats)
            print_clock_accuracy(stats)

        if ms > tick_end:
            break

    cleanup_device_under_test(device)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # TODO close USB port cleanly
        pass
