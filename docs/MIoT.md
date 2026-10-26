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

The [Xiaomi tower fan p45 preset](autoexecs/Xiaomi-Smart-Tower-Fan-2-p45.bat)
maps the fan controls and enables Home Assistant discovery. Copy its contents
into the device's LittleFS `autoexec.bat` after selecting the correct UART. This
is OpenBeken's startup command format. The controls have been exercised through
Home Assistant against the stock MCU image; the board's UART assignment and
physical behavior still require verification.

Use channel commands or `miotSet SIID PIID Value` to request a write. `miotSet`
requires literal `true` or `false` for a boolean; channel writes use zero or one.
`miotRead SIID PIID` requests a fresh read and `miotPoll` refreshes every mapping.
For example, queue `miotSet 2 1 true` before `miotSet 2 5 37`: the p45 MCU ignores
speed and oscillation changes while power is off, even when its result says success.

## Home Assistant

On builds with MQTT and Home Assistant discovery enabled, bind four existing
writable mappings with:

```text
miotFan PowerChannel SpeedChannel OscillationChannel AngleChannel
```

The binding creates a fan with power, percentage, and oscillation, plus a swing
angle select on the same device. Power and oscillation must be boolean; speed
must be an integer range starting at one or greater with a step of one. The
angle mapping must provide between two and sixteen integer options. Invalid
bindings, including those exceeding the discovery payload limit, leave the
previous configuration unchanged. The transport remains generic: SIID/PIID
values come from the selected mappings, not a device model.

Discovery uses the normal OpenBeken device identity, discovery prefix, and MQTT
availability topic. Commands use `miotSet` without MQTT retention. Setting fan
percentage to zero turns power off; turning on with a percentage sends power
before speed.

Consecutive unsent writes to the bound speed mapping keep only the latest value.
An in-flight write still completes its independent readback. Other property
writes retain their order and separate speed updates, so coalescing cannot move
a speed change across a power or mode change. This applies to both `miotSet` and
channel commands while the fan binding is enabled.

Optional enum and timer bindings extend the same device:

```text
miotFanEnum preset ModeChannel Label...
miotFanEnum profile ProfileChannel Label...
miotFanTimer DurationChannel RemainingChannel
```

Enum bindings accept up to four distinct labels, ordered from the mapped integer
minimum to maximum with unit steps. Use underscores for spaces in labels. Presets
appear on the fan entity, alongside speed and oscillation; a profile is an associated
select. Each binding must use distinct mappings. The p45 preset provides Direct,
Natural, and Sleep modes and four natural-wind profiles. The profile mapping also
represents stepped speed in Direct mode; use it with the appropriate operating mode.

Timer bindings require matching integer ranges starting at zero, with unit steps.
Duration is writable and remaining time is read-only, both in minutes. Setting the
timer requests the MCU's countdown; zero cancels it. The p45 preset writes duration
directly. Its separate delay-enable property is unnecessary and would overwrite
the requested duration with a one-hour preset. Writes while powered off may be
acknowledged without changing the MCU; readback remains authoritative.

The stock p45 MCU fixture verifies modes, natural profiles, and timer behavior.
Native driver selftests pass normally and with AddressSanitizer. The production
BL602 core-control bridge regression also passes. Home Assistant integration
verifies the extended controls, timer expiry/cancellation, and restart recovery.

Home Assistant's standard fan popup exposes presets and oscillation. Angle, profile,
timer, and remaining time use associated entities that can share one dashboard panel.

The retained `<client-id>/miot/fan/get` JSON snapshot contains `power`, `speed`,
`oscillation`, and `angle`, plus `mode`, `profile`, `timer`, and `remaining` for their
optional bindings. It is published only after every bound mapping has
MCU-confirmed values. A successful write result or a speculative local channel
change does not establish confirmed state. Discovery and the latest confirmed
snapshot are republished after MQTT reconnect. `miotFan off` or stopping the
driver removes its retained discovery and state; mappings and channel control
remain available after `miotFan off`.

Confirmed updates are coalesced into one snapshot at the end of the quick tick.
The slow tick retries pending publication; periodic reads remain a recovery
fallback. Commands and write acknowledgments alone do not trigger publication.

MQTT availability describes the module's broker connection, not MCU or motor
health. Until readback arrives, a retained snapshot represents the last known
MCU state.

## State and recovery

Network replies use `uap` while the setup AP is active, `cloud` with station Wi-Fi
and MQTT connected, and `local` with Wi-Fi alone. Disconnected devices report
`offline` if either saved SSID exists, otherwise `unprov`. AP mode takes precedence
over the shared Wi-Fi connected flag. The MCU's `cloud` state enables unsolicited
p45 button/timer notifications; the broker can be entirely local.

Changes coalesce until the next command poll, which sends `MIIO_net_change` after
any post-write readback. A direct `net` reply also synchronizes status; driver and
MCU restarts force resynchronization.

POSIX time requests use synchronized NTP when available; other time requests
return zero. Firmware update, factory reset, string/float properties, and actions
are not implemented. Events are acknowledged without a channel mapping.

Commands wait for the MCU's `get_down` poll. Background reads rotate and share
polls with queued writes; each write is followed by an independent read, including after an error or
a missing result. Success replies alone never establish channel state. MCU change
notifications and successful reads update channels without echoing another write.
OpenBeken's ordinary channel commands initially change the local channel value;
readback reconciles it when the MCU leaves the requested value unchanged.
Rejected channel values are reconciled on the next driver tick, after the outer
GPIO and event callbacks finish. Incoming updates suppress their direct echo;
automations can still request another value on that same channel.

Outgoing values are checked against the configured type, bounds, and step.
Incoming values must match their type and bounds; a value between advertised steps
is retained when the MCU reports it. A malformed multi-property notification is
rejected as a whole. Unknown notifications are acknowledged and ignored.

The driver bounds its mappings, queue, and line buffer. A full queue rejects a new
entry but permits replacing its last unsent fan speed. A transaction timeout
schedules a read without retrying the write, because an error can still have
side effects. Partial or oversized frames are discarded
through the next CR. An MCU `model` announcement clears queued pre-reset commands
and schedules fresh reads. Periodic reads also refresh mapped state. Stopping the
driver or handing its UART to another driver prevents further UART processing.

## Network recovery

MCU-requested Wi-Fi reset is disabled whenever the driver starts. Add
`miotAllowNetworkReset 1` after `startDriver MIoT` in the startup script to opt in.
Use `miotAllowNetworkReset 0` to reject future requests. An accepted request already
scheduled for reboot remains pending.

When enabled, the MCU's `restore` command without arguments is acknowledged and
clears both saved Wi-Fi networks through the normal configuration setters. The
existing deferred save and reboot path returns the module to AP setup. MQTT,
channel settings, property mappings, and LittleFS startup scripts are preserved.
The MCU may reset its own fan settings independently of the module.

Queued control writes are discarded, and new writes are rejected until reboot.
Repeated `restore` requests are acknowledged without clearing settings again or
postponing reboot. MCU identification and fresh reads can still resynchronize
channel state. Full factory erase and firmware updates remain outside this
handler. Until reboot, an accepted reset reports `unprov`. Physical Wi-Fi indicator
behavior still requires board verification.

## Verification

The host self-tests exercise framing, strict input validation, notification
atomicity, readback, errors, silent ignores, timeout recovery, backpressure, restart,
UART ownership, network-reset policy, and saved configuration reload. A separate
Renode bridge runs the compiled host driver against the unmodified p45 MCU image
for the four core controls and button-triggered network recovery. Its parent-owned
harness observes reboot/AP callbacks and reloads simulator flash in a fresh process;
radio behavior and BL602 EasyFlash persistence need separate verification.
An isolated Home Assistant/Mosquitto lab exercises real discovery, services,
confirmed state, entity identity, and recovery through this host/MCU bridge.
The BL602 build checks target compilation; physical pins, electrical timing, motor
behavior, and running the driver on BL602 silicon require separate verification.
