# MIoT serial MCU driver

The `MIoT` driver connects OpenBeken channels to Xiaomi MCU properties over the
module's selected UART. It uses CR-terminated ASCII at 115200 baud, 8N1. Select the
correct UART with the existing UART configuration flag before starting the driver;
the board's wiring determines the pins. The driver is enabled for BL602 and the
host simulator.

## Configuration

Start the driver, then configure each property with:

```text
miotMap SIID PIID Channel bool|int Min Max Writable [Step]
```

`Writable` is `0` for a read-only mapping and `1` for a writable mapping. `Step`
defaults to one and constrains outgoing values relative to `Min`. Boolean mappings
require bounds zero and one. Property identifiers and channels must be unique.
Mappings are recreated by the startup script after reboot.

For the Xiaomi tower fan p45, the following core controls have been tested against
the stock MCU firmware running in an emulator:

```text
startDriver MIoT
miotMap 2 1 1 bool 0 1 1
miotMap 2 5 2 int 1 100 1
miotMap 2 6 3 bool 0 1 1
miotMap 2 7 4 int 30 150 1 30
```

These map power, speed percentage, oscillation, and swing angle to channels 1–4.
Configure channel types and labels with the usual OpenBeken commands. This driver
does not register a Home Assistant fan entity.

Use channel commands or `miotSet SIID PIID Value` to request a write. `miotSet`
requires literal `true` or `false` for a boolean; channel writes use zero or one.
`miotRead SIID PIID` requests a fresh read and `miotPoll` refreshes every mapping.
For example, queue `miotSet 2 1 true` before `miotSet 2 5 37`: the p45 MCU ignores
speed and oscillation changes while power is off, even when its result says success.

## State and recovery

The driver answers startup identification, network, and polling requests. It
reports Wi-Fi as local or offline, without claiming Xiaomi cloud connectivity.
POSIX time requests use synchronized NTP when available; other time requests
return zero. Firmware update, factory reset, string/float properties, and actions
are not implemented. Events are acknowledged without a channel mapping.

Commands wait for the MCU's `get_down` poll. Reads take priority over queued
writes; each write is followed by an independent read, including after an error or
a missing result. Success replies alone never establish channel state. MCU change
notifications and successful reads update channels without echoing another write.
OpenBeken's ordinary channel commands initially change the local channel value;
readback reconciles it when the MCU leaves the requested value unchanged.

Outgoing values are checked against the configured type, bounds, and step.
Incoming values must match their type and bounds; a value between advertised steps
is retained when the MCU reports it. A malformed multi-property notification is
rejected as a whole. Unknown notifications are acknowledged and ignored.

The driver bounds its mappings, queue, and line buffer. A full queue rejects a new
write. A transaction timeout schedules a read without retrying the write, because
an error can still have side effects. Partial or oversized frames are discarded
through the next CR. An MCU `model` announcement clears queued pre-reset commands
and schedules fresh reads. Periodic reads also refresh mapped state. Stopping the
driver or handing its UART to another driver prevents further UART processing.

## Verification

The host self-tests exercise framing, strict input validation, notification
atomicity, readback, errors, silent ignores, timeout recovery, backpressure, restart,
and UART ownership. A separate Renode bridge runs the compiled host driver against
the unmodified p45 MCU image for the four core controls and recovery checks.
The BL602 build checks target compilation; physical pins, electrical timing, motor
behavior, and running the driver on BL602 silicon require separate verification.
