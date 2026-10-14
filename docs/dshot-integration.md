# DShot and passthrough integration plan

Updated 2026-10-03 against esp32-dshot commit
`4713b423977dbf0df34b8af64580afffe27446f1`.
The reusable session wrapper now implements the passthrough-side callbacks,
lazy UART activation/cleanup and retained per-channel reboot tracking described
below. See [the component API](component-api.md). The minimal DShot300 application
now lives in `examples/dshot` and uses the
current `dshot::Passthrough` helper. Its managed Git dependency is pinned to
that published revision. Physical handoff, reboot waveform and the boot/arming interval
still need bench validation. Motor restoration policy belongs to the example
application, not this reusable component.

## Fixed behavior for this integration

The combined ESP32 firmware creates both components and supplies their shared
ESC pin list. In this document, "firmware" means that code, not the browser
configurator. Passthrough handles MSP, 4-way and the ESC bootloader protocol;
DShot handles motor frames. Neither library needs to depend on the other.

All ESCs switch together. Stop every motor driver and close every motor link
before passthrough can use any ESC pin. There is no option to keep another
motor link running during configuration.

There are two MSP requests with different jobs:

| Valid request | Firmware action before a successful reply |
|---|---|
| Empty MSP_API_VERSION, command 1 | Call acquire: stop all DShot output and close all links. Hold configuration mode. |
| Empty MSP_SET_PASSTHROUGH, command 245 | Install/activate the ESC UART. If discovery was skipped, call acquire first. |

"Read Settings" is a browser button, not a separate firmware trigger. The
inspected [ESC Configurator handlers][esc-app] send discovery on Connect and
request 4-way entry when reading settings. UART activation happens on receipt
of MSP 245. Waiting for that request avoids starting ESC communication for a
connection that only performs discovery. DShot is already stopped at MSP 1
and stays stopped throughout. No button detection or selectable entry policy
is needed. A cable connection or open serial port alone does not trigger entry.

Validate the complete packet, checksum and empty payload before any lifecycle
action. Other MSP information requests do not acquire the pins by themselves.

## Use the existing DShot API

| API | Relevant behavior |
|---|---|
| Link::set(0) | Publishes zero and cancels command/EDT sequences. Frames continue, so this does not release the pin. |
| Driver::stop() | Joins the worker after its active exchange finishes or is aborted. Closes the GPTimer and cancels sequences; RMT remains allocated. |
| Link::end() | Stops its driver, closes TX/RX RMT resources, detaches the link and disables its GPIO. Safe to repeat. |
| Link::begin() | Opens fresh RMT resources with an initial zero payload. Constructors do not touch hardware. |
| Driver::start() | Creates the repeating worker after the links are open. Later worker errors are reported by last_error(). |

Use stop/end and begin/start. The new `dshot::Passthrough` helper wraps those
operations and rejects DShot reopening while held; the example uses it. No
partial-link operation or transport redesign is required. The bench
console's stop command only publishes zero, so do not use it for this handoff.

Stopping alone retains an open link's payload, but this plan closes and reopens
all links, which clears it. The firmware also clears its own remembered throttle,
arming timer and pending motor requests when entering configuration mode.
Reject new motor/ESC-command requests in that mode rather than saving them to
execute after exit. No request-generation scheme is needed for this example.

DShot cleanup currently calls abort() if an IDF resource cleanup operation
fails. For example, a failed rmt_disable() during Link::end() enters fatal
error handling rather than returning an acquire error. Retain that behavior
for this integration; it prevents proceeding into passthrough after incomplete
DShot cleanup. Returning recoverable cleanup errors would require a separate
DShot API change and is not a prerequisite here.

## Session callbacks

Add session callbacks to the reusable ESP-IDF wrapper, separate from the
existing pt_esc_ops_t.release backend hook:

```c
typedef struct {
    esc_session_end_reason_t reason;
    uint8_t bootloader_attempted_mask;
    uint8_t reboot_sent_mask;
    uint8_t needs_reboot_mask;
    bool io_released;
} esc_session_end_t;

typedef struct {
    void *ctx;
    esp_err_t (*acquire)(void *ctx);
    esp_err_t (*release)(void *ctx, const esc_session_end_t *result);
} esc_passthrough_lifecycle_t;
```

These types are now declared in `include/esc_passthrough.h`. End reasons distinguish normal
interface exit, transport loss, session timeout, setup failure, reply failure
and explicit shutdown. Bit N refers to channel N in the shared pin list.
The result pointer is valid during the callback; copy it if it is needed later.
Keep ESP-IDF types out of the portable protocol layer and add C linkage guards
to public C headers used by the C++ firmware.

Both callbacks run synchronously in task context. One firmware task owns
lifecycle calls and Link setters. If the parser runs in another task, send its
lifecycle request to that owner and wait for completion. The owner must not be
blocked waiting for the parser that is waiting for it.

### acquire: shut down all DShot output

Call acquire once on the first valid MSP 1, before its successful reply. A
direct MSP 245 calls it first if configuration mode has not yet been acquired.
Repeated discovery and repeated MSP 245 do not call it again while held.

The callback blocks motor requests, clears the firmware's remembered requests
and arming timer, calls stop() on every motor driver, and calls end() on every
motor link. Return ESP_OK only after those calls finish. Merely queuing stop
does not grant pin access. A returned error produces an MSP error response and
no UART activation; the callback must leave any partially stopped output
stopped. No release callback is owed for an unsuccessful acquisition.

### release: report session completion, then conditionally restore DShot

Call release exactly once for each successful acquisition when the session
ends, including setup/reply failure after acquisition. Passthrough first
finishes or aborts its active transaction, attempts required reboots, and shuts
down its UART and GPIO access. It then reports the result to the firmware.

The callback restarts zero-frame DShot only for a normal exit with
io_released == true and needs_reboot_mask == 0. For other results it records
the reason and leaves all DShot output stopped. ESP_OK means the result was
handled; it does not mean that motor output was restarted. A DShot reopen/start
error is returned and recorded, and leaves output stopped.

If passthrough cleanup fails, set io_released to false and report it without
claiming that the pins are free. Do not start DShot or free memory still used
by a UART task/IRQ. Retain the remaining resources for cleanup or enter fatal
error handling if they cannot safely be retained. A new session must not
acquire those pins until cleanup has completed.

The backend release hook only releases the selected ESC wire/connection. It
is used during channel changes and individual errors too. It must never call
the firmware's release callback or restart DShot.

## Entry and operation

1. On valid MSP 1, acquire all ESC pins through the callback, then answer
   discovery. The UART remains inactive and motor requests remain blocked.
2. On valid MSP 245, acquire first if needed, install/configure the UART and
   claimed GPIOs, then send the successful channel-count reply. Enable 4-way
   processing only after that reply succeeds. Setup failure sends an MSP error
   if the host transport still works. Repeated entry is harmless.
3. If setup or either entry reply fails after acquisition, clean up acquired
   passthrough resources and call release with the failure reason. Leave DShot
   stopped; never pretend passthrough entry succeeded.
4. Handle bootloader operations normally. Channel changes, individual read
   errors and backend wire release leave the configuration session held.
5. A new valid discovery during an existing 4-way session resets the protocol
   to discovery without releasing the acquisition or restarting DShot. Retire
   the active wire operation and retain reboot tracking across this reconnect.
   Repeated discovery while already in discovery simply answers it.

Use one shared ordered pin list. The number of advertised passthrough channels
must match the motor links managed by the integration. Do not independently
configure four passthrough channels and fewer DShot links in the combined example.

## ESC reboot tracking belongs to passthrough

Keep bootloader state for every channel for the entire held session, not just
the currently selected ESC:

- Before attempting bootloader entry, set that channel's bit in
  bootloader_attempted_mask and needs_reboot_mask. A lost reply does not prove
  that the ESC stayed out of the bootloader.
- Switching channels, failed reads, keepalive failure and backend release do
  not clear those bits.
- Handle browser reset requests (4-way 0x35) through the backend. Clear a
  needs_reboot bit only when the backend's operation to return that ESC to
  firmware succeeds. A reset that leaves it in bootloader mode is insufficient.
  Record a completed reboot in reboot_sent_mask; a later bootloader entry sets
  needs_reboot again.
- On session exit, the passthrough wrapper asks the backend to reboot every
  channel still marked needs_reboot, while it still owns the pins. Skip ESCs
  already rebooted by the browser unless they were entered again. Attempt all
  remaining channels even if one reboot fails. Report uncleared bits to the
  firmware instead of restarting DShot.

The inspected [configurator disconnect handler][esc-app] sends ESC reset
requests before interface exit. Tracking also covers missing reset requests,
failed identification and channels omitted from the browser's settings list.

The original backend's reset result only reported wire selection/transmission.
The pulse_low hook now returns success/failure, and the UART transport checks
GPIO errors before reporting waveform completion. Validate the reset/reboot
sequence on the supported Bluejay ESC before using it for motor restoration.
reboot_sent means that sequence completed, not that firmware startup was
independently confirmed. Do not claim a universal boot-ready check from the
current backend, and do not require bidirectional telemetry for restoration.

## Normal exit and restoration

Accept a valid interface exit (4-way 0x34) whenever configuration mode is held,
including Connect followed by Disconnect without MSP 245. That case has no
UART to close and no entered ESCs to reboot. An exit with no held session is
harmless and does not call release again.

For a held session:

1. Stop accepting new ESC operations and finish the active transaction. Send
   the existing exit reply without making the browser wait for reboot/arming.
   The inspected [4-way client][esc-fourway] does not wait for a parsed exit
   reply. A reply transmission failure still triggers cleanup, but is reported
   as reply failure and does not automatically restore DShot.
2. Reboot every ESC still marked needs_reboot. Complete all reset waveforms
   before relinquishing the pins.
3. Release backend connections, detach UART matrix routes from every shared
   pin, uninstall the UART driver and retire task/IRQ access. Report whether
   this completed in io_released.
4. Call release once with the end reason and reboot results. Clear the protocol
   session; retain unresolved reboot results in the wrapper/firmware so a later
   reconnect does not erase them.
5. For a normal exit with released IO and no pending reboots, the firmware
   reopens every link, explicitly sets zero, then starts every motor driver.
   If any begin/set/start fails, stop all drivers, close partly reopened links
   and record the error. Do not leave a subset running.
6. Send only zero frames during the supported ESC's boot/arming interval.
   The firmware owns that deadline. Nonzero run requests and ESC commands
   during configuration, restoration or this interval return
   ESP_ERR_INVALID_STATE and are not queued. Afterward, require a fresh request;
   never restore old throttle. The callback can return after starting zero
   frames; the owner task handles the deadline without blocking the parser.
7. Keep checking Driver::last_error(), as the current DShot bench does. A later
   worker failure stops/closes all motor links and is reported to the firmware.
   Driver::start() success does not confirm ESC startup or future transmissions.

For this first example, use a documented boot/arming interval validated on the
bench ESC. Preserve the existing DShot configuration. If changing ESC firmware
or settings changes signaling/EDT requirements, update the driver's configuration
before restoration; Driver holds a configuration snapshot.

## Loss, timeout and shutdown

On cable loss, failed host output, session timeout or explicit firmware shutdown,
finish/abort any current transaction, attempt remaining reboots while local
ESC IO is available, retire UART/GPIO access, and call release once with that
reason. All DShot output remains stopped even if reboot attempts succeeded.
Explicit firmware recovery is required before resuming zero frames and the
boot/arming interval. Recovery must resolve any pending reboots and confirm IO
release first. Preserve that pending state across reconnects; a new session
must not turn a previous failed reboot into a clean exit by forgetting it.

Refresh a configurable session lease on valid MSP/4-way traffic, including
keepalives. Do not expire during an active operation; allow long ESC operations
and idle UI periods. The existing 250 ms parser timeout discards partial frames
only. Neither timeout automatically restarts motors.

Detect physical USB loss where supported. IDF 6.1's
usb_serial_jtag_is_connected() reports host SOF activity even when the serial
port is closed, so it cannot reliably identify a closed browser tab/port.
Use protocol exit, explicit firmware shutdown, output failure, reconnect
discovery and the session lease to cover those cases. A lease expiry releases
passthrough resources and leaves motor output stopped.

## Required implementation changes

- Implemented: `src/esc_uart.c/.h` belongs to the reusable root component.
  Configuration no longer touches hardware. UART/GPIO activation is lazy and
  shutdown checks errors and retires all routes; wire release remains separate.
- Implemented: session wrapper, callbacks, end report, persistent reboot tracking
  and exactly-once cleanup. Discovery is validated before ownership changes;
  device operations outside 4-way are rejected and exit works during discovery.
- Standalone passthrough now lives in `examples/basic` with omitted owner callbacks.
  Implemented: a C++ integration example using the current DShot handoff helper.
  One firmware task owns both lifecycles and all motor requests. The example
  fixes regular DShot300, one link and binary USB controls. Its separate DShot
  dependency is fetched from Git rather than a sibling checkout.
- Give binary USB serial one owner. Disable the DShot bench REPL/log output on
  that stream or move it to another UART. Reports to firmware must not print
  text into MSP/4-way traffic. MSP idle/status responses must agree with the
  combined firmware's actual state.
- No settings-address restrictions or DShot transport redesign are introduced
  by this integration. Distribution still requires resolving DShot's licence.

## Implementation and validation phases

1. Implement the wrapper and lazy UART lifecycle. Host tests cover validation
   before callbacks; acquire/stop/end before replies or UART activation; direct
   MSP 245; repeated discovery/entry/exit; failed acquire/setup/reply; and no
   restart on channel changes, backend release or abnormal session end.
2. Test reboot tracking across multiple channels, failed bootloader entry,
   browser reset, re-entry after reset, omitted reset and failed reboot. Assert
   reboot before UART shutdown before release before DShot reopen. Cover exit
   without 4-way, cleanup failure, reconnect with unresolved reboot state,
   partial restoration rollback and rejection of motor requests until ready.
3. Create a C3 example with one link on GPIO 0. Bench check zero-frame DShot ->
   Connect/MSP 1 -> no DShot pulses -> MSP 245 -> settings read/write -> exit ->
   reboot as needed -> zero-frame DShot -> boot/arming interval -> fresh run.
   Also Connect/Disconnect without reading, repeat sessions, entry during
   TX/telemetry, and unplug/reconnect. Use a logic analyser to confirm exclusive
   pin use and verify that abnormal exits leave DShot stopped.
4. Exercise two C3 links, then four S3 links and bidirectional telemetry. All
   configured links switch together. Current DShot limits are two links on C3
   and four on S3; passthrough uses one UART sequentially. Four simultaneous
   DShot links on C3 would require a separate resource design, outside this plan.

[esc-app]: https://github.com/stylesuxx/esc-configurator/blob/21b407a3f30aaff1b079496f4d13d82544c667c4/src/Containers/App/index.jsx
[esc-fourway]: https://github.com/stylesuxx/esc-configurator/blob/21b407a3f30aaff1b079496f4d13d82544c667c4/src/utils/FourWay.js
