"""
test_connection.py
Quick test to verify the Crazyradio PA can find and connect to the Crazyflie.
Run this BEFORE trying any flight scripts.

Usage:
    python test_connection.py
"""

import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# The URI your scan found — change if different
URI = "radio://0/80/2M/E7E7E7E13"


def main():
    # Initialize the low-level drivers (including Crazyradio)
    cflib.crtp.init_drivers()
    print(f"Drivers initialized. Scanning...")

    # Scan for available Crazyflies
    available = cflib.crtp.scan_interfaces()
    print(f"Found {len(available)} Crazyflie(s):")
    for cf in available:
        print(f"  {cf[0]}")

    if not available:
        print("\nNo Crazyflie found! Check that:")
        print("  1. Crazyradio PA dongle is plugged in")
        print("  2. Crazyflie is powered on (green LEDs)")
        print("  3. Try unplugging and replugging the dongle")
        return

    # Try to connect
    uri = available[0][0]  # Use the first one found
    print(f"\nConnecting to {uri} ...")

    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
            print("Connected!")

            # Read some basic parameters to verify comms
            cf = scf.cf
            print(f"\nReading parameters...")

            # Log battery voltage
            log_config = None
            try:
                from cflib.crazyflie.log import LogConfig
                log_config = LogConfig(name="Battery", period_in_ms=500)
                log_config.add_variable("pm.vbat", "float")

                battery_readings = []

                def battery_callback(timestamp, data, logconf):
                    voltage = data["pm.vbat"]
                    battery_readings.append(voltage)
                    print(f"  Battery: {voltage:.2f}V")

                cf.log.add_config(log_config)
                log_config.data_received_cb.add_callback(battery_callback)
                log_config.start()

                # Read for 3 seconds
                time.sleep(3)
                log_config.stop()

                if battery_readings:
                    avg = sum(battery_readings) / len(battery_readings)
                    print(f"\nAvg battery voltage: {avg:.2f}V")
                    if avg < 3.0:
                        print("WARNING: Battery is low! Charge before flying.")
                    elif avg > 4.0:
                        print("Battery is fully charged.")
                    else:
                        print("Battery level is OK for testing.")

            except Exception as e:
                print(f"Could not read battery: {e}")

            print("\nConnection test PASSED!")

    except Exception as e:
        print(f"\nConnection FAILED: {e}")
        print("\nTroubleshooting:")
        print("  - Unplug and replug the Crazyradio dongle")
        print("  - Power cycle the Crazyflie")
        print("  - Make sure cfclient GUI is NOT also running")


if __name__ == "__main__":
    main()
