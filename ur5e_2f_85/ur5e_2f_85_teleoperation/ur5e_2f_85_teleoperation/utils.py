ACTIVE=1
INACTIVE=0

import glob
import os

def find_dualshock_hidraw():
    """
    Returns the /dev/hidraw* path for the first connected PS4 controller (DualShock 4)
    or None if not found.
    """
    VENDOR_ID = "054C"   # Sony
    PRODUCT_ID = "09CC"  # DualShock 4 USB

    for hid in glob.glob("/dev/hidraw*"):
        try:
            # Read the uevent file for this hidraw device
            uevent_path = f"/sys/class/hidraw/{os.path.basename(hid)}/device/uevent"
            with open(uevent_path, "r") as f:
                content = f.read()
                print(content)
            if VENDOR_ID in content and PRODUCT_ID in content:
                return hid
        except Exception:
            continue
    return None
