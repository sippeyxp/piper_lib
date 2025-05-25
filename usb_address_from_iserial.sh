#!/bin/bash

# Script to find a USB device by its iSerial using lsusb -v

# Function to display usage
usage() {
    echo "Usage: $0 <iSerial_number>"
    echo "  Looks for a USB device with the specified iSerial number."
    echo "  Reports its USB address (e.g. 3-5.4:1.0) if found."
    echo ""
    echo "Example: $0 A104E9J0"
    exit 1
}

# --- Main Script ---

# Check if an argument is provided
if [ -z "$1" ]; then
    echo "Error: No iSerial number provided."
    usage
fi

TARGET_ISERIAL="$1"

# Iterate through all USB devices in /sys/bus/usb/devices
for devpath in /sys/bus/usb/devices/*; do
    # Only consider directories with a colon (":") in their name (interfaces), or skip to all
    # Actually, we want device directories (e.g. 3-5.4), not interfaces (which have :1.0 etc)
    # So, skip if the path is not a directory or if it contains a colon
    if [[ ! -d "$devpath" ]]; then
        continue
    fi
    devname=$(basename "$devpath")
    # Skip interface subdirs (which have a colon)
    if [[ "$devname" == *:* ]]; then
        continue
    fi
    # Check if iSerial file exists
    if [[ -f "$devpath/serial" ]]; then
        iserial=$(<"$devpath/serial")
        if [[ "$iserial" == "$TARGET_ISERIAL" ]]; then
            # Now, find the first interface (should be devname:1.0 or similar)
            # If no interface, just print devname
            iface=""
            for ifacepath in "$devpath":*; do
                if [[ -d "$ifacepath" ]]; then
                    iface=$(basename "$ifacepath")
                    break
                fi
            done
            if [[ -n "$iface" ]]; then
                echo "$iface"
            else
                # If no interface, just print device name
                echo "$devname"
            fi
            exit 0
        fi
    fi
done

echo "No USB device with iSerial '$TARGET_ISERIAL' found."
exit 1
