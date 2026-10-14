# DShot example

Control one motor with regular DShot300, then use the same signal pin
for ESC configuration.

## Build

With ESP-IDF 6.1 activated, from the repository root:

```sh
cd examples/dshot
idf.py set-target esp32c3 # or esp32s3
idf.py menuconfig
idf.py build
idf.py -p YOUR_SERIAL_PORT flash
```

Set the signal GPIO under `DShot passthrough example`.

## Motor controls

Install `pyserial` if needed. From the repository root:

```sh
python3 tools/dshot_control.py --port YOUR_SERIAL_PORT status
python3 tools/dshot_control.py --port YOUR_SERIAL_PORT arm
python3 tools/dshot_control.py --port YOUR_SERIAL_PORT run 75
python3 tools/dshot_control.py --port YOUR_SERIAL_PORT stop
```

`arm` waits five seconds before throttle is accepted. `run` takes a raw
DShot value from 48 to 2047. The motor keeps running after the tool exits;
use `stop` to return to zero throttle.

## Switch to the configurator

Close the control tool before connecting the configurator. Connecting
stops DShot and releases the signal pin for passthrough.

Read or change settings, then use the configurator's Disconnect button.
After a successful disconnect, DShot restarts at zero throttle and arms
for five seconds. Send a new `run` command to move the motor again.

USB loss or a failed session leaves motor output stopped in `fault`.

This example has build and host tests. Its physical handoff and reboot
timing still need testing on hardware.
