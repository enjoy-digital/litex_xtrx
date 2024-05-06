#!/usr/bin/env python3

import argparse
import subprocess

# RMS Test -----------------------------------------------------------------------------------------

def rms_test(channel, filename):
    # Capture channel.
    capture_status = subprocess.run(
        ["./adrv902x_record", "-c", str(channel), filename, "0x10000"],
        stdout = subprocess.DEVNULL,  # Suppress the stdout.
        stderr = subprocess.STDOUT
    ).returncode
    if capture_status != 0:
        return

    # Check channel.
    tone_check_result = subprocess.run(
        ["./tone_check.py", filename],
        stdout = subprocess.PIPE,
        text   = True
    ).stdout
    print(f"Channel {channel}:\n{tone_check_result}")

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="RX RMS Test on all SDR channels.")
    args = parser.parse_args()

    for channel in range(4):
        rms_test(channel=channel, filename=f"tone_rx{channel}.bin")

if __name__ == "__main__":
    main()