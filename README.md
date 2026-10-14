# ESP32 ESC Passthrough

Use an ESP32-C3 or ESP32-S3 to read and change ESC settings through
ESC Configurator or [AM32 Configurator](https://am32.ca).

Supports up to four ESCs. Requires ESP-IDF 6.1 and one free hardware UART.

Tested with Bluejay v0.21 on C3 and AM32 v2.21 on S3, including a
SEQURE_4IN1_F421.

DISCLAIMER: I'd probably only use this to read and write ESC settings i do not trust my own abilities enough to flash firmware with this just yet.

## Add to your project

In `main/idf_component.yml`:

```yaml
dependencies:
  esp32-esc-passthrough:
    git: https://github.com/aroncullberg/esp32-esc-passthrough.git
    version: main
```

Include `esc_passthrough.h`. See the [API](docs/component-api.md) for
configuration and callbacks to hand over pins from your motor driver.

## Examples

- [Basic adapter](examples/basic/README.md): USB passthrough for up to four ESCs.
- [DShot](examples/dshot/README.md): motor control and passthrough on one signal pin.

## Tests

From the repository root:

```sh
python3 tests/run_tests.py
```
