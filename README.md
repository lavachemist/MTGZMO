# Machine Tool Gizmo

A Raspberry Pi Pico 2W firmware that reads a quadrature encoder, displays live RPM on a GC9A01 round TFT display using LVGL 8, drives a stepper motor via a TMC5160 at a configurable ratio of the measured RPM, and controls a Yaskawa A1000 VFD over MEMOBUS/Modbus RTU. Designed for hobbing machine synchronisation — the stepper speed tracks the spindle speed according to a user-defined hob thread / gear tooth count.

> ⚠️ **The TMC5160 stepper output has not been hardware-tested yet.** The ratio calculations, VMAX register math, and driver initialisation are implemented but unverified on a real motor. Do not rely on the stepper output for machine synchronisation until it has been validated.

## Features

- Live RPM tachometer gauge on a 240×240 GC9A01 round display (LVGL 8, needle + digital readout)
- Startup sweep animation
- Exponentially smoothed RPM reading from a quadrature encoder (direction-aware)
- TMC5160 stepper driver in StealthChop velocity mode — speed tracks encoder RPM via hob/gear ratio, corrected for belt/pulley gearing
- Stepper driver abstraction — swap to a different driver by replacing two functions only
- Yaskawa A1000 VFD control over MEMOBUS/Modbus RTU (RS-485): run forward, run reverse, stop, set speed, fault reset
- Physical VFD controls: Start/Stop toggle button, Reverse button, Stop button, potentiometer for continuous speed adjustment (0–max Hz), fail-safe lockout jumper for web controls (GPIO 22)
- VFD fault monitoring — on-screen overlay on any new trip, and on fault clearance
- **Safety**: the VFD is force-stopped on every controller boot/reset (a crash or reflash never leaves the spindle running unattended); the web lockout jumper is enforced server-side and fails safe (absent/disconnected = locked, not just greyed-out buttons); all state-changing web endpoints reject cross-site requests (CSRF) via an Origin check — Stop is never blocked by either mechanism
- WiFi web UI at **`http://gizmo.mt`** (AP mode) to adjust all parameters at runtime
- AP mode SSID `MTGizmo` — no router needed; captive portal auto-opens browser on connect
- Station mode support with automatic fallback to AP if connection fails
- Physical button (GPIO 14): short press shows current IP on display for 5 s, long press (2 s) forces AP mode
- On-screen overlay notifications for all WiFi mode transitions and VFD faults
- All runtime settings persist across reboots via EEPROM
- Onboard LED blinks at 1 Hz while the VFD is running, 0.25 Hz while stopped

## Hardware

- Raspberry Pi Pico 2W (RP2350)
- GC9A01 240×240 round TFT display
- Rotary encoder (quadrature, channel A interrupt + channel B for direction)
- BigTreeTech TMC5160 v1.2 stepper driver
- 2-phase stepper motor
- External motor PSU (8–40 V)
- SparkFun RS-485 Breakout (or any MAX485-compatible half-duplex transceiver)
- Yaskawa A1000 VFD
- 6× momentary push buttons / switches (GPIO 4, 5, 6, 14, 16, 22)
- 2 kΩ–100 kΩ potentiometer (GPIO 26 / ADC0) — 10 kΩ recommended

## Wiring

**Pico 2W (RP2350) — main controller**

![Raspberry Pi Pico 2W pinout](assets/pico2w-pinout.svg)

**Pico (RP2040) — A1000 simulator**

![Raspberry Pi Pico pinout](assets/pico-pinout.svg)


### GC9A01 Display → Pico 2W (SPI0)

| GC9A01 Pin | Pico 2W GPIO | Pico 2W Pin |
|---|---|---|
| SCLK | GPIO 18 | Pin 24 |
| MOSI | GPIO 19 | Pin 25 |
| CS | GPIO 17 | Pin 22 |
| DC | GPIO 20 | Pin 26 |
| RST | — | Not connected |
| BL | 3.3 V | Pin 36 (or PWM GPIO for dimming) |
| VCC | 3.3 V | Pin 36 |
| GND | GND | Any GND |

### Rotary Encoder → Pico 2W

| Encoder Pin | Pico 2W GPIO | Pico 2W Pin |
|---|---|---|
| Channel A | GPIO 2 | Pin 4 |
| Channel B | GPIO 3 | Pin 5 |
| VCC | 3.3 V | Pin 36 |
| GND | GND | Any GND |

> Channel B is sampled inside the channel-A ISR for direction detection. Both channels are pulled up internally. Direction can be software-reversed in the web UI without rewiring.

### BTT TMC5160 v1.2 → Pico 2W (SPI1)

| Pico 2W GPIO | Pico 2W Pin | BTT TMC5160 v1.2 Pin |
|---|---|---|
| GPIO 10 | Pin 14 | SCK |
| GPIO 11 | Pin 15 | SDI |
| GPIO 12 | Pin 16 | SDO |
| GPIO 13 | Pin 17 | CS |
| GPIO 15 | Pin 20 | EN |
| 3.3 V | Pin 36 | VIO |
| GND | Any GND | GND |

> **VM** on the TMC5160 connects to your external motor PSU (8–40 V), **not** the Pico.  
> GND must be common between the Pico and the motor PSU.  
> EN is driven HIGH (disabled) during startup and pulled LOW (enabled) once the driver is fully configured.

### Motor → BTT TMC5160 v1.2

Connect your 2-phase stepper motor coils to the **A1/A2** and **B1/B2** terminals on the TMC5160 board.

### SparkFun SP3485 Breakout → Pico 2W (UART1)

The SparkFun SP3485 is a 3.3 V RS-485 transceiver breakout. It is pin-compatible with the MAX485 but operates natively at 3.3 V — no level shifting required.

| SP3485 Breakout Pin | Direction | Pico 2W GPIO | Pico 2W Pin |
|---|---|---|---|
| RX-I (driver input) | Pico → breakout | GPIO 8 (UART1 TX) | Pin 11 |
| TX-O (receiver output) | Breakout → Pico | GPIO 9 (UART1 RX) | Pin 12 |
| DE | Transmit enable | GPIO 7 | Pin 10 |
| VCC | Power | 3.3 V | Pin 36 |
| GND | Ground | GND | Any GND |

> **RX-I / TX-O naming is from the breakout's perspective relative to the RS-485 bus** — the opposite of what you'd expect from the Pico side. RX-I is the breakout receiving from the Pico (Pico TX). TX-O is the breakout transmitting to the Pico (Pico RX). Swapping these two wires is the most common wiring mistake and will produce zero response bytes from the drive.

> DE is driven HIGH to transmit and LOW to receive. The firmware handles this automatically around every Modbus transaction. The SP3485 breakout ties DE and RE together on-board, so a single GPIO controls both.

### RS-485 → Yaskawa A1000 VFD

| RS-485 Breakout Pin | A1000 Terminal |
|---|---|
| A (+) | S+ (R+) |
| B (−) | S− (R−) |

> Terminate with a 120 Ω resistor across A/B at the far end of the cable if the bus is long or noisy.

### Buttons → Pico 2W

All buttons wire the same way: one leg to the GPIO, other leg to GND. All use the internal pullup — no external resistors needed.

| Button | Pico 2W GPIO | Pico 2W Pin | Function |
|---|---|---|---|
| WiFi / IP | GPIO 14 | Pin 19 | Short press: show IP on display 5 s · Long press (≥2 s): force AP mode |
| VFD Start | GPIO 4 | Pin 6 | Press sends Run Forward command |
| VFD Reverse | GPIO 5 | Pin 7 | Press sends Reverse command |
| VFD Stop | GPIO 6 | Pin 9 | Press sends Stop command |
| RPM Source | GPIO 16 | Pin 21 | Hold LOW to show VFD RPM on display; open (HIGH) for encoder RPM |
| Web Lockout | GPIO 22 | Pin 29 | Jumper to GND (LOW) to *unlock* web VFD controls; open/disconnected (HIGH) fails safe to *locked* |

### Potentiometer → Pico 2W

| Pot terminal | Connection | Pico 2W Pin |
|---|---|---|
| Left (GND end) | GND | Any GND |
| Wiper (centre) | GPIO 26 (ADC0) | Pin 31 |
| Right (3.3 V end) | 3.3 V | Pin 36 |

> Any 2 kΩ–100 kΩ linear pot works; 10 kΩ recommended. The wiper voltage is read by the 12-bit ADC (forced via `analogReadResolution(12)`) and mapped from `VFD_POT_MIN` (100 counts) to 4095 → 0–max frequency. Full CCW maps to 0 Hz. Deadband is 32 counts (~0.5 Hz) to suppress ADC noise oscillation. While the pot is active (ADC > 50), the web speed input is greyed out in the UI.

## Configuration

Compile-time constants at the top of [`src/main.cpp`](src/main.cpp):

| Define | Default | Description |
|---|---|---|
| `MOTOR_STEPS` | `200` | Full steps per revolution of your stepper |
| `MICROSTEPS` | `256` | TMC5160 microstep resolution |
| `TMC_RMS_CURRENT` | `1000` | Motor RMS current limit in mA |
| `TMC_R_SENSE` | `0.075` | Sense resistor in ohms (0.075 Ω for BTT TMC5160 v1.2) |
| `METER_MAX_RPM` | `4000` | Maximum RPM shown on the gauge |
| `RPM_UPDATE_MS` | `50` | RPM recalculation interval in milliseconds |

Runtime-adjustable settings (survive reboot via EEPROM):

| Setting | Default | Description |
|---|---|---|
| Hob threads | `1` | Number of starts on the hob cutter |
| Gear teeth | `32` | Number of teeth on the gear being cut |
| Encoder PPR | `600` | Encoder pulses per revolution (single channel) |
| Encoder reversed | `false` | Flip encoder direction without rewiring |
| Stepper driver pulley | `1 : 1` | Driver and driven pulley tooth counts for belt correction |
| VFD slave address | `1` | A1000 Modbus slave address (matches H5-01) |
| VFD max frequency | `60.00 Hz` | Upper clamp on any speed setpoint sent to the drive |
| VFD baseline frequency | `60.00 Hz` | Drive output frequency that corresponds to baseline RPM |
| VFD baseline RPM | `1750` | Motor shaft RPM at the baseline frequency |
| WiFi mode | AP | AP or Station |
| Station SSID / password | — | Router credentials for Station mode |

## Hobbing Ratio & Pulley Gearing

### Hobbing ratio

The required output shaft speed (gear blank) relative to the encoder (spindle/hob) is:

```
output_rpm = encoder_rpm × (hob_threads / gear_teeth)
```

**`hob_threads`** is the number of starts (threads) on the hob cutter.  
**`gear_teeth`** is the number of teeth on the gear being cut.

For every full rotation of the output shaft (gear blank), the encoder (spindle/hob) must complete `gear_teeth / hob_threads` rotations.

### Stepper pulley / belt correction

If the stepper drives the output shaft through a belt-and-pulley reduction, the stepper must spin faster or slower than the required output RPM. The firmware applies the belt ratio automatically so the **final output always matches the hobbing ratio**, regardless of pulley size:

```
stepper_rpm = output_rpm × (pulley_driven / pulley_driver)
```

Combined into a single formula:

```
stepper_rpm = encoder_rpm × (hob_threads / gear_teeth) × (pulley_driven / pulley_driver)
```

**`pulley_driver`** is the tooth count on the pulley attached to the stepper shaft.  
**`pulley_driven`** is the tooth count on the pulley attached to the output shaft.

Set both to `1` (default) when the stepper drives the output directly with no belt.

### Examples

| Hob threads | Gear teeth | Driver pulley | Driven pulley | Behaviour |
|---|---|---|---|---|
| 1 | 32 | 1 | 1 | Direct drive — spindle turns 32× per 1 blank rotation |
| 1 | 40 | 1 | 1 | Direct drive — spindle turns 40× per 1 blank rotation |
| 2 | 40 | 1 | 1 | 2-start hob — spindle turns 20× per blank rotation |
| 1 | 32 | 20 | 40 | Belt 1:2 reduction — stepper spins at 2× output RPM |
| 1 | 32 | 40 | 20 | Belt 2:1 step-up — stepper spins at 0.5× output RPM |

## VFD Control (Yaskawa A1000)

The firmware acts as a Modbus RTU master on UART1 at 9600 bps, 8-N-2. The A1000 is polled every 500 ms to read the status word (register `0x0020`) and fault code (`0x0021`).

### A1000 setup

Set these parameters on the A1000 before connecting:

| Parameter | Value | Description |
|---|---|---|
| H5-01 | 1 | Slave address — must match web UI VFD Settings |
| H5-02 | 3 | Baud rate = 9600 bps |
| H5-03 | 0 | Data format = 8-N-2 |
| b1-01 | 2 | Frequency reference source = Modbus |
| b1-02 | 2 | Run command source = Modbus |

### Fault notifications

When a new fault is detected the fault name is shown on the display overlay for 6 s. When the fault clears, "VFD Fault Cleared" is shown for 3 s. Common faults decoded on-device:

| Code | Name |
|---|---|
| 0x01 | oC — Overcurrent |
| 0x02 | ov — Overvoltage |
| 0x03 | oH1 — Heatsink overheat |
| 0x05 | oL1 — Motor overload |
| 0x06 | oL2 — Drive overload |
| 0x0B | EF — External fault |
| 0x0C | EF0 — Modbus-triggered fault |
| 0x11 | LF — Output phase loss |
| 0x17 | CE — Modbus communication error |
| 0x1F | UV1 — DC bus undervoltage |
| 0x23 | CPF — Control circuit fault |

See [`src/a1000_modbus.json`](src/a1000_modbus.json) for the complete register map and fault code list.

## Web UI

Connect to the `MTGizmo` WiFi network (password: `hobbing1`). The captive portal should auto-open in your browser; if not, navigate to **`http://192.168.4.1`** manually.

| Setting | Value |
|---|---|
| SSID | `MTGizmo` |
| Password | `hobbing1` |
| IP (AP mode) | `192.168.4.1` |

The UI has two tabs:

### Home tab

- **VFD — Yaskawa A1000** card — shows live VFD status (Running / Stopped / No comms), current speed, and setpoint in RPM. Controls: RPM input + **Set RPM**, **▶ Forward**, **◀ Reverse**, **Stop ■**, **Reset** (fault reset). All controls except Stop (including speed input) are disabled in the UI — and rejected by the server, even via a direct request — unless the lockout jumper (GPIO 22) is present. The RPM is converted to a frequency command using the baseline scaling and capped at `vfd_max_hz` before sending via Modbus. Status auto-refreshes every 500 ms.
- **Gear Ratio** card — enter *Threads on hob* and *Gear teeth*; the display shows the current ratio as `H : T`. Takes effect immediately and persists to EEPROM.

### Settings tab

- **Encoder** card — toggle to reverse the encoder direction without rewiring.
- **Encoder PPR** card — set pulses per revolution (1–10 000). Presets: 100, 200, 400, 600, 1000, 2400.
- **Stepper Pulley** card — set driver and driven pulley tooth counts. Set both to `1` for direct drive.
- **VFD Settings** card — set the Modbus slave address (1–31), maximum frequency clamp (Hz), baseline frequency (Hz), and RPM at baseline frequency. All persist to EEPROM.
- **WiFi** card — switch between AP and Station mode. In Station mode, click **Scan** to populate a dropdown of visible networks (or manually type the SSID); enter password and apply. If connection fails within 20 s the device falls back to AP mode automatically. The Station credentials are preserved even after a fallback.

> Captive portal (auto-open browser) only works in AP mode. In Station mode use the IP address shown in the WiFi card, or press the physical button (GPIO 14) to display it on the round display for 5 s.

### Web API endpoints

| Endpoint | Method | Parameters | Description |
|---|---|---|---|
| `/` | GET | — | Serve the web UI |
| `/set` | POST | `hob`, `teeth` | Set hobbing ratio |
| `/set-encoder` | POST | `reversed` (0/1) | Set encoder direction |
| `/set-ppr` | POST | `ppr` | Set encoder PPR |
| `/set-pulley` | POST | `driver`, `driven` | Set pulley tooth counts |
| `/set-wifi` | POST | `mode`, `ssid`, `pass` | Configure WiFi |
| `/wifi-scan` | GET | — | Scan for WiFi networks; returns JSON array of SSIDs |
| `/vfd-rpm` | POST | `rpm` | Set VFD speed in RPM (converted to Hz, capped at vfd_max_hz) — locked, CSRF-checked |
| `/vfd-run` | POST | — | Send run (forward) command to VFD — locked, CSRF-checked |
| `/vfd-reverse` | POST | — | Send run (reverse) command to VFD — locked, CSRF-checked |
| `/vfd-stop` | POST | — | Send stop command to VFD — always allowed, never locked or CSRF-checked |
| `/vfd-freq` | POST | `hz` (0.01 Hz units) | Set VFD frequency reference directly in 0.01 Hz units — locked, CSRF-checked |
| `/vfd-reset` | POST | — | Reset active VFD fault — locked, CSRF-checked |
| `/vfd-settings` | POST | `slave`, `maxhz`, `basehz`, `baserpm` | Set slave address, max/baseline frequency, baseline RPM — locked, CSRF-checked |
| `/vfd-status` | GET | — | Returns JSON: `{comms_ok, running, reverse, status, fault, freq, output_freq, base_hz, base_rpm, pot_active, web_lock}` |

> **CSRF protection**: every state-changing endpoint above (all `/set*` and `/vfd*` POST routes except `/vfd-stop`) rejects the request with `403` if the browser-supplied `Origin` header doesn't match this device's own address — this stops a malicious or compromised page open in another tab from silently issuing commands to the mill. Requests with no `Origin` header at all (curl, scripts) are still allowed through, since there's no auth/token scheme to fall back on.
>
> **Locked** above means the endpoint also returns `403` unless the lockout jumper (GPIO 22) is present — see [Web lockout jumper](#web-lockout-jumper-gpio-22).

## Physical Controls

### WiFi / IP button (GPIO 14)

| Press | Action |
|---|---|
| Short press (< 2 s) | Shows the current IP address on the display for 5 s |
| Long press (≥ 2 s) | Forces a switch to AP mode (`MTGizmo` / `192.168.4.1`) |

### VFD Start button (GPIO 4)

Sends Run Forward command. Edge-triggered — holding the button does not repeat.

### VFD Reverse button (GPIO 5)

Sends a single Reverse command (command word `0x0002`) on each press. Edge-triggered.

### VFD Stop button (GPIO 6)

Sends Stop command. Edge-triggered.

### RPM source select (GPIO 16)

Hold LOW to display VFD RPM on the tachometer (read from the drive's output frequency register). Leave open (HIGH, default) to display encoder RPM. Switching between sources shows a brief "RPM: VFD" or "RPM: Encoder" overlay.

### Web lockout jumper (GPIO 22)

`INPUT_PULLUP`, and **fail-safe**: the pin must be actively pulled LOW — by a jumper physically bridging it to GND — to *unlock* the web VFD controls. Any absence of that connection (jumper not installed, wire broken, connector unplugged) reads HIGH and defaults to *locked*. This is the opposite of a normal "hold to disable" switch on purpose — a safety interlock should fail toward the safe state, not the enabled one.

While locked: the speed input, Set RPM, Forward, Reverse, and Reset are all greyed out and non-functional in the web UI — and rejected server-side with `403` even if requested directly (e.g. via `curl`), so the jumper is a real interlock, not just a UI hint. The hardware pot and physical buttons remain operational, and **Stop always works regardless of lockout state**. The UI shows "Controls locked — unlock jumper not present" below the buttons while locked.

### VFD speed potentiometer (GPIO 26 / ADC0)

Sampled every 50 ms. The 12-bit ADC (0–4095) is mapped from `VFD_POT_MIN` (100 counts, configurable) to 4095 → 0–`vfd_max_hz`. Full counterclockwise = 0 Hz. Only sends a new Modbus write when the reading changes by more than 32 ADC counts (~0.5 Hz hysteresis), preventing oscillation from ADC noise. While the pot ADC reads > 50 counts, the web speed input and Set RPM button are greyed out in the UI with a "Speed controlled by potentiometer" note.

## Stepper Driver Abstraction

The TMC5160-specific code is isolated in two functions inside a clearly marked section of [`src/main.cpp`](src/main.cpp):

- **`stepper_init()`** — hardware setup, SPI configuration, StealthChop enable
- **`stepper_set_rpm(float rpm)`** — translates a signed RPM value into VMAX register writes

To substitute a different driver (e.g. a step/dir driver like a DRV8825), replace only those two functions plus the `#include`, pin defines, and driver object. Everything else — ratio calculation, web UI, EEPROM, display — is untouched.

## EEPROM Layout

Settings are stored in 148 bytes of emulated EEPROM:

| Bytes | Content |
|---|---|
| 0 | Magic byte (`0xB0`) — detects valid data |
| 1 | WiFi mode (0 = AP, 1 = STA) |
| 2–65 | Station SSID (null-terminated, max 63 chars) |
| 66–129 | Station password (null-terminated, max 63 chars) |
| 130–131 | `hob_threads` (uint16_t, little-endian) |
| 132–133 | `gear_teeth` (uint16_t, little-endian) |
| 134 | Encoder reversed (0 or 1) |
| 135–136 | Encoder PPR (uint16_t, little-endian) |
| 137–138 | `pulley_driver` (uint16_t, little-endian) |
| 139–140 | `pulley_driven` (uint16_t, little-endian) |
| 141 | VFD slave address (uint8_t) |
| 142–143 | VFD max frequency (uint16_t, 0.01 Hz units, little-endian) |
| 144–145 | VFD baseline frequency (uint16_t, 0.01 Hz units, little-endian) |
| 146–147 | VFD baseline RPM (uint16_t, little-endian) |

> The magic byte is checked on every boot. If it does not match `0xB0`, all settings revert to firmware defaults and EEPROM is re-initialised on next save.

## A1000 Modbus RTU Simulator (`pico_simulator`)

[`src/simulator.cpp`](src/simulator.cpp) implements a second firmware target that turns a standard Raspberry Pi Pico (RP2040) into a Yaskawa A1000 stand-in on the RS-485 bus. It is useful for bench-testing the main firmware without a real VFD.

### What it simulates

- Modbus RTU slave at address `1`, 9600 bps, 8-N-2
- **FC 03** Read Holding Registers
- **FC 06** Write Single Register
- **FC 08** Loopback diagnostic (sub-function 0x0000 only)

| Register | Access | Description |
|---|---|---|
| `0x0001` | R/W | Command word — bit 0 = run forward, bit 1 = run reverse, bit 3 = fault reset |
| `0x0002` | R/W | Frequency reference (0.01 Hz units) |
| `0x0020` | R | Status word — mirrors running / reverse / fault / ready bits |
| `0x0021` | R | Fault code (0 = no fault) |
| `0x0025` | R | Output frequency — ramps toward frequency reference when running |

All other registers within the normal A1000 address range return `0x0000` on read; writes to unknown registers return Modbus exception 02 (Illegal Data Address).

The simulated drive ramps its output frequency at ~0.5 Hz/s toward the commanded reference, giving a realistic response to acceleration/deceleration commands.

The onboard LED blinks at **1 Hz** while the simulated drive is running, and **0.25 Hz** while stopped. Both firmware targets use the same blink rates so the behaviour is consistent across the bench setup.

### Simulator wiring (MAX485 breakout → standard Pico)

Same pinout as the main controller — connect both boards to the same A/B bus pair:

| Pico GPIO | Pico Pin | MAX485 Pin | Direction |
|---|---|---|---|
| GPIO 8 (Serial2 TX) | Pin 11 | DI | Pico → MAX485 |
| GPIO 9 (Serial2 RX) | Pin 12 | RO | MAX485 → Pico |
| GPIO 7 | Pin 10 | DE + RE (tied) | Transmit enable |
| 3.3 V | Pin 36 | VCC | — |
| GND | Any GND | GND | — |

### Building and flashing the simulator

```
pio run -e pico_simulator
pio run -e pico_simulator --target upload
```

`pio run` (without `-e`) only builds the `pico2w` main firmware (`default_envs = pico2w`).

## Serial Debug Monitoring

Both firmware targets print VFD activity to USB Serial at **115200 baud**. Connect via USB and open a serial monitor to observe all Modbus transactions in real time.

```
pio device monitor -b 115200
```

### Main firmware (`pico2w`) log messages

| Prefix | When printed |
|---|---|
| `[VFD] init …` | Once at boot, after `vfd_init()` configures UART1 |
| `[VFD TX] 01 06 …` | Every frame sent to the drive (raw hex bytes) |
| `[VFD RX] 01 06 …` | Every frame received from the drive (raw hex bytes) |
| `[VFD RX] timeout — no bytes` | Response window elapsed with no data |
| `[VFD] CMD run-forward` | `vfd_run()` called (button, web UI, or API) |
| `[VFD] CMD run-reverse` | `vfd_reverse()` called |
| `[VFD] CMD stop` | `vfd_stop()` called |
| `[VFD] CMD fault-reset` | `vfd_reset_fault()` called |
| `[VFD] CMD set-freq 6000 (60.00 Hz)` | `vfd_set_freq()` called (potentiometer or web UI) |
| `[VFD] FC06 reg=0x… val=0x…  OK` | Write single register succeeded |
| `[VFD] FC06 reg=0x… val=0x…  FAIL (…)` | Write failed — reason in parentheses |
| `[VFD] FC03 reg=0x… cnt=2  OK  [0]=0x…  [1]=0x…` | Read holding registers succeeded, values shown |
| `[VFD] FC03 reg=0x… cnt=2  FAIL (…)` | Read failed — reason in parentheses |
| `[VFD] poll  status=0x…  fault=0x… (…)  running=…` | Printed whenever the polled status or fault code changes |
| `[VFD] poll — comms lost` | First poll failure after a previously successful comms state |

### Simulator (`pico_simulator`) log messages

| Prefix | When printed |
|---|---|
| `[SIM] A1000 simulator ready …` | Once at boot |
| `[SIM RX] 01 03 …` | Every frame received from the master (raw hex bytes) |
| `[SIM] bad CRC — discarded` | Frame received with invalid CRC |
| `[SIM] FC=0x03 slave=1` | Decoded function code and slave address |
| `[SIM] FC03 read reg=0x… cnt=…` | FC 03 read request decoded |
| `[SIM] FC06 write reg=0x… val=0x…` | FC 06 write request decoded |
| `[SIM] FC06 reg=0x… — illegal address, sending exception` | Write to unimplemented register |
| `[SIM TX] 01 03 …` | Every response frame sent back to the master (raw hex bytes) |

> Only changes to the VFD status word or fault code are logged during polling — steady-state polls that return the same values produce no output, keeping the monitor readable.

## Building

Built with [PlatformIO](https://platformio.org/). Open the project folder and run:

```
pio run
```

To build and upload:

```
pio run --target upload
```

### Dependencies (resolved automatically by PlatformIO)

| Library | Version |
|---|---|
| `lvgl/lvgl` | ~8.3.11 |
| `Bodmer/TFT_eSPI` | ^2.5.43 |
| `teemuatlut/TMCStepper` | ^0.7.3 |
| `DNSServer` | (bundled with arduino-pico) |
| `WebServer` | (bundled with arduino-pico) |
| `WiFi` | (bundled with arduino-pico) |
| `EEPROM` | (bundled with arduino-pico) |

Platform: `https://github.com/maxgerhardt/platform-raspberrypi.git`
Board: `rpipico2w`

## Reference Files

- [`src/a1000_modbus.json`](src/a1000_modbus.json) — complete Yaskawa A1000 MEMOBUS/Modbus register map: command/status bit definitions, all monitor registers, parameter addresses, fault codes, example raw byte sequences, and scaling reference.
- [`pico-arduino/`](pico-arduino/) — original simpler tachometer prototype (Pico, not 2W; encoder-only; no stepper, WiFi, VFD, or EEPROM). Kept as a reference.
