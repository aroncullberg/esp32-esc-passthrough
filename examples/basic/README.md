# Basic adapter

USB passthrough for up to four ESCs.

## Build

With ESP-IDF 6.1 activated, from the repository root:

```sh
cd examples/basic
idf.py set-target esp32c3 # or esp32s3
idf.py menuconfig
idf.py build
idf.py -p YOUR_SERIAL_PORT flash
```

Set the signal GPIOs under `ESC passthrough`. Enable ESC slots in order,
starting at ESC 1, with no gaps or duplicate pins.

Select a free UART under `Component config → ESC passthrough`.

## Wiring and connection

Connect each ESC signal to its configured GPIO and connect the ESP and
ESC grounds. Power the ESCs separately.

Connect the ESP's native USB port to your computer. In the configurator,
use flight-controller passthrough mode; leave Direct off in AM32.

Connect, then read the settings. The first read can take a few seconds
while the ESCs enter their bootloaders. Use the configurator's Disconnect
button when finished.

chrome might not show it as a port, in that case you're SOL, sorry have not found a fix for that yet.
