# Avoiding a hardware UART

Investigated 2026-10-03 against the installed ESP-IDF 6.1 tree
(`v6.1-1-g048d2ec12de`) and the sources below. The current transport remains the
bench-tested hardware UART design, now activated lazily and configurable.
No software UART implementation is claimed or enabled.

**A UART-free ESC transport appears feasible. RMT is the preferred next
prototype.** It uses a peripheral for timed pulses but no UART controller.
Pure CPU/GPIO software UART is possible in principle, with tighter latency
constraints. Neither changes the number of signal pins: the existing transport
already shares one bidirectional signal GPIO per ESC and one UART across all
ESCs. This addresses scarce UART controllers, rather than saving extra GPIOs.

## Requirements from this backend

- 19200 baud, 8N1, one-wire half-duplex; drive only during TX and reboot pulse.
- Bit duration: 52.083 us. A byte takes 520.833 us.
- Largest TX: 256 data bytes plus two CRC bytes, about 134.375 ms.
- Largest RX: 256 data bytes plus CRC/ACK, about 134.896 ms.
- Fast TX-to-RX release for the first reply; avoid local echo.
- Bounded receive waits, including deliberate no-reply detection after SET_BUFFER.
  Erase may wait up to three seconds. An absent ESC must not trap the CPU.
- Sequential channel selection, observable 300 ms reboot pulse and complete
  route/IRQ/task cleanup before returning pins to DShot.

These numbers come from the local production backend, not measured edge timing.

## Options

| Option | Assessment |
| --- | --- |
| IDF dedicated-GPIO soft_uart example | Demonstrates C3/S3 support, but requires substantial redesign for this protocol. |
| GPIO-edge ISR timestamps plus byte decoder | Feasible at 19200; depends on worst-case IRQ latency and CPU-driven TX timing. |
| RMT TX + RX with software UART encoder/decoder | Preferred prototype: hardware-timed bits/captures, finite waits, no UART controller. |
| Borrow an application UART | Smaller fallback when a hardware UART may be suspended, but interrupts the other user. |

Espressif's [dedicated-GPIO example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/dedicated_gpio/soft_uart)
uses IRAM assembly and disables interrupts during each send/receive. Its README
disables both watchdogs for that behavior. Inspection of the installed 6.1
`soft_uart.c` and assembly also shows fixed baud choices starting at 115200,
separate TX/RX pin bundles, and no receive timeout: it waits for every requested
byte. Copying it unchanged would risk roughly 135 ms of interrupt suppression
for a large packet, or an indefinite wait for an absent ESC. A rewrite needs
19200 timing, half-duplex pin release, finite deadlines and bounded scheduling.

[EspSoftwareSerial](https://github.com/plerup/espsoftwareserial) provides another
approach: its ISR records edges/timestamps and byte assembly happens later.
Its authors document timing errors under competing interrupt activity. It is
an Arduino library, so adopting it in this IDF-only component would require
adapting dependencies and validating both target chips. It is evidence that
an interrupt-driven software design is plausible, rather than a ready-made
replacement for this backend.

## RMT prototype design

The [IDF RMT driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/rmt.html)
provides TX encoders, RX pulse capture, receive streaming, callbacks and channel
retirement. The local C3/S3 SoC headers specify 48 symbols per channel and RX
ping-pong support; S3 also supports DMA. A 259-byte response will not fit in one
48-symbol capture, so streaming/ISR servicing is essential on C3.

A design to validate:

1. Allocate one TX and one RX channel after DShot closes all links. Reuse this
   pair sequentially for every ESC; free it before the owner release callback.
   Other application RMT users can still cause allocation failure. Two TX/two
   RX channels exist on C3 and four/four on S3 in IDF's resource mapping.
2. Use a fixed RMT clock and accumulated fractional bit ticks for 19200 timing.
   Encode start/data/stop bits in software; the peripheral emits the waveform.
3. Prepare RX state and switch from drive to receive at TX completion in a
   bounded ISR path. The existing DShot transport demonstrates calling
   rmt_receive from its TX-completion callback. UART replies require their own
   turnaround and idle timing validation. Ensure input/output routes both
   relinquish the pin; handle and suppress local echo.
4. Decode captured level durations into UART bytes with start/stop validation,
   including long runs of equal bits, idle gaps and partial captures. A worst
   case of roughly five symbols per byte gives about 1295 symbols / 5180 bytes
   for a full response, plus margin. Use a sufficiently sized capture buffer or
   copy/decode partial chunks before they are overwritten.
5. Service RX ping-pong interrupts during the whole reply, with finite operation
   deadlines and explicit overflow/error handling. Cancel on timeout and retire
   callbacks before releasing storage. Keep watchdogs enabled and do not mask
   interrupts for an entire packet.
6. Use checked GPIO drive/release for reboot pulses and the existing backend
   command framing. Expose it through `esc_passthrough_config_t.io`; no MSP,
   bootloader or motor-owner API redesign is needed.

This is an engineering inference from the driver capabilities and current
handoff, not a validated transport. RMT reduces the CPU timing burden but still
needs prompt interrupt servicing for long RX bursts on C3.

## Acceptance before switching the default

Build both targets and test the codec with all byte values, 0x00/0xFF/0x55 runs,
fragmented captures, invalid stop bits, inter-byte gaps and maximum packet sizes.
Then compare repeated settings reads with the current UART on the Bluejay
fixture, including disconnected ESCs, SET_BUFFER's no-reply interval, writes
with read-back, reboot, multiple channels and repeat sessions. Measure the
first reply bit, baud error, high-impedance turnaround and callback retirement
with a logic analyser. Repeat under USB and the intended application's Wi-Fi/
interrupt load. Keep the current UART default until these measurements pass.

The reusable component now has lazy IO start/stop and a custom wire transport
interface. This leaves both a software implementation and UART borrowing
possible without adding another owner callback today. The investigation does
not establish that a pure GPIO design can meet every application's latency
budget, nor that RMT will always be free when passthrough starts.
