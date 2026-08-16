# A1000 MEMOBUS/Modbus reference: what was wrong and what changed

`src/a1000_modbus.json` was fact-checked against Yaskawa's official **A1000 Technical Manual (SIEP C710616 41H)**, downloaded directly from yaskawa.com. The previous version of the file — kept for reference as [`src/a1000_modbus_old.json`](a1000_modbus_old.json) — turned out to have several inaccuracies, some cosmetic and some that matter for real hardware. This document explains what was checked, what was wrong, and what it means for this project's firmware (`src/main.cpp`).

## How this was verified

- Every example Modbus frame's CRC-16 was independently recomputed byte-for-byte (script-verified, not eyeballed) against both the old and new files.
- Register and bit-field content was checked against the manual's actual text — Appendix C ("MEMOBUS/Modbus Communications"), Table C.5 (Command Register Data), Table C.6 (Fault Trace Contents), Table C.7 (Alarm Register Contents), Section C.4 (Setup Parameters), and Section C.11 (Communication Errors) — extracted directly from the PDF, not summarized by a third party.
- Anything **not** independently re-confirmed against the manual in this pass is explicitly marked `"verified": false` (or carries a `verification_note`) in the new file, rather than presented as checked.

## What was correct (no change needed)

- **All CRC-16 math.** Every example frame's checksum in the old file was already correct.
- **Command register `0001H`, bits 0–3** (Forward Run, Reverse Run, External Fault EF0, Fault Reset).
- **`H5-01`** (slave address, range/default), **`H5-04` through `H5-07`** (comm-error stopping method, fault detection, TX wait time, RTS control).
- **`b1-01` / `b1-02` = 2 required for Modbus control** — confirmed verbatim in the manual's Table C.2.
- Frequency reference in `0.01 Hz` units — correct, but only as long as parameter `o1-03` is left at its default (0). The new file states this conditionally instead of unconditionally.

## What was wrong

### 1. Status register `0020H` — almost entirely wrong (matters for real hardware)

The old file's bit map for the drive's primary status word was fabricated past bit 0. Real bit map (Technical Manual p.717):

| Bit | Old file said | Actually is |
|---|---|---|
| 0 | During Run | During Run ✓ |
| 1 | Zero Speed | **During Reverse** |
| 2 | Reverse Running | **Drive Ready** |
| 3 | Reset | **Fault** |
| 4 | Speed Agree | Data Setting Error |
| 5 | Drive Ready | Multi-Function Contact Output 1 |
| 6 | Minor Fault | Multi-Function Contact Output 2 |
| 7 | Major Fault | Multi-Function Contact Output 3 |
| 11 | Freq Ref from Modbus | Reserved |
| 12 | Run Cmd from Modbus | Reserved |
| 14 | Freq Ref Loss | ComRef enabled |
| 15 | Reserved | ComCtrl enabled |

**Firmware impact:** `vfd_poll()` in `main.cpp` reads bit 2 (`new_status & 0x0004`) expecting "Reverse Running." On a real A1000 that bit is **Drive Ready**, which reads 1 almost any time the drive is healthy — not direction information at all. The real "During Reverse" bit is bit 1. This session's hardware-in-the-loop testing used a simulator built against the *old, wrong* spec, so firmware-vs-simulator agreement never caught this — it only proved the two matched each other, not that either matched a real A1000.

### 2. Fault reporting — the file modeled a register that doesn't work the way it assumed

The old file treated register `0021H` as a single enumerated fault code (`0x00`–`0x3A` → `oC`, `ov`, `oH1`, ...). It is not. `0021H` is **"Fault Contents 1,"** a 16-bit *bitmask* of fault categories (bit 0 = Overcurrent/Ground Fault, bit 7 = EF–EF8, etc. — Technical Manual p.717).

The real single-number fault code Yaskawa intends integrators to read lives at a **different register entirely: `0080H`** (parameter `U2-01`, "Current Fault"), with its own numbering scheme (Table C.6) — e.g. `0007H` = Overcurrent, not `0x01`. There's also a *third*, separate "Minor Fault Code" register at `007FH` (Table C.7) with yet another numbering.

**Firmware impact:** `vfd_fault_name()` in `main.cpp` decodes register `0021H` against a switch/case table built for neither of the real numbering schemes. On an actual drive fault, it will show "Unknown fault" or, worse, a plausible-but-wrong name. The new JSON's `fault_and_alarm_codes` section lays out all three real mechanisms with their real codes; `main.cpp` would need to switch to reading `0080H` and use the corrected table to actually work correctly.

### 3. Output frequency — `main.cpp` is reading the wrong monitor register

The old file listed `0025H` as "Output Frequency." The manual's register table (p.718) **and** the description of parameter `H5-10` ("Unit Selection for MEMOBUS/Modbus Register 0025H") both independently confirm `0025H` is actually **Output Voltage Reference**. Real Output Frequency is `0024H`.

**Firmware impact:** `main.cpp` reads a block of 6 registers starting at `0x0020` and takes index 5 (`0x0025`) as `vfd_output_freq`. On real hardware that's actually reading the output voltage (roughly hundreds, in 0.1 V units) and treating it as a frequency (expecting thousands, in 0.01 Hz units). This feeds the "VFD RPM" tachometer display source and the `/vfd-status` web JSON — both would show bogus values on a real A1000. The simulator matched the same wrong assumption, so this also went undetected in this session's testing.

### 4. Command register `0001H`, bits 4–13 — fabricated fixed meanings

Old file assigned fixed functions (Multi-step Ref, Jog, Accel/Decel Select, Baseblock, Torque Reference, Emergency Stop, KEB, Drive Enable). Real manual: bits 4–11 are generic, user-configurable **Multi-Function Inputs 1–8** (whatever you assign via `H1-01`..`H1-08`); bits 12–13 are Reserved. Not a current firmware bug (`main.cpp` never writes those bits), but wrong reference info for anyone extending it.

### 5. Registers `0022H` and `0023H` mislabeled

Old file called `0022H` "Alarm Code" (it's actually **Data Link Status** — EEPROM-write/limit-error bits) and `0023H` "Drive Status 2" with a made-up bitmask (it's actually just the **Frequency Reference** monitor value; the real Drive Status 2 bitmask lives at `002CH`).

### 6. `H5-02` and `H5-03` — incomplete/fabricated ranges

- `H5-02` (baud rate): old file listed range `0–5`. Real range is `0–8` — it's missing 57600/76800/115200 bps. Harmless here since the firmware only uses setting 3 (9600), but incomplete.
- `H5-03`: old file claimed a 4-way parity **and** stop-bit encoding (`8-N-2`/`8-E-1`/`8-O-1`/`8-N-1`). The real `H5-03` only controls parity (range `0–2`: none/even/odd) — there's no separate stop-bit selector in the manual. `main.cpp`'s `SERIAL_8N2` still happens to be correct, because it matches the standard Modbus RTU convention of 2 stop bits when no parity is used — not because of anything `H5-03` explicitly sets.

### 7. Exception response codes — generic Modbus codes swapped in for A1000-specific ones

Old file used the standard/generic Modbus exception codes (`01`=Illegal Function, `02`=Illegal Data Address, `03`=Illegal Data Value, `04`=Slave Device Failure, `05`=Acknowledge, `06`=Slave Device Busy). The real A1000 (Technical Manual p.733) uses its own set: `01`/`02`/`03` are conceptually similar (Function Code / Register Number / Bit Count errors) but the drive also defines `21H`–`25H` (Data Setting Error, Write Mode Error, DC Bus Undervoltage Write Error, Write Error During Parameter Process, Writing into EEPROM Disabled) that the old file didn't have at all, and codes `04`–`06` don't exist on the A1000.

### 8. `parameter_registers` — systematic per-group address offsets

A follow-up pass checked every remaining entry against Technical Manual Appendix B (the parameter address tables: B.3 for `A1`, B.4 for `b1`, B.5 for `C1`, B.6 for `d1`/`d2`, B.7 for `E1`/`E2`). Several were wrong by a consistent offset *per parameter group* — not random noise, which is a strong sign they were interpolated/guessed rather than read off a real table:

| Parameter | Old file said | Actually is | Offset |
|---|---|---|---|
| `A1-02` | `0100H` | `0102H` | −2 |
| `A1-03` | `0101H` | `0103H` | −2 |
| `C1-01` | `0201H` | `0200H` | +1 |
| `C1-02` | `0202H` | `0201H` | +1 |
| `C1-03` | `0203H` | `0202H` | +1 |
| `C1-04` | `0204H` | `0203H` | +1 |
| `d2-01` | `0291H` | `0289H` | +8 |
| `d2-02` | `0292H` | `028AH` | +8 |
| `E2-01` | `0310H` | `030EH` | +2 |
| `E2-02` | `0311H` | `030FH` | +2 |
| `E2-03` | `0312H` | `0310H` | +2 |
| `E2-04` | `0313H` | `0311H` | +2 |
| `E2-05` | `0314H` | `0312H` | +2 |

`b1-01` (`0180H`), `b1-02` (`0181H`), `b1-04` (`0183H`), `d1-01`–`d1-04` (`0280H`–`0283H`), and `E1-01`/`E1-04`/`E1-05`/`E1-09` (`0300H`/`0303H`/`0304H`/`0308H`) were all already correct.

**Impact:** low in practice for this project specifically — `main.cpp` never writes any of these parameter addresses at runtime (it only relies on `b1-01`/`b1-02` being set correctly on the drive itself, via the front panel, which this file's addresses don't affect either way). But anyone scripting a parameter backup/restore tool or a first-time-setup wizard against the *old* file's addresses would have silently written to the wrong parameter.

Every entry in `parameter_registers` is now individually verified; the section-level `"verified"` flag was updated from `false` to `true`. `required_parameter_setup`'s `b1-01`/`b1-02` addresses were also confirmed correct and flipped from `"verified": false` to `true`.

## What's still unverified

Everything in the file has now been checked against the manual, with one exception: the exact decimal precision of the Output Power monitor (register `0027H`) — the manual's table lists its unit as `kW` without stating a scaling factor the way it does for current (`0.1 A`) or torque (`0.1%`), and no other page in the manual sections reviewed clarified it. `scaling_summary.power` is left without a `"verified"` flag for this reason; treat `0.1 kW` as a reasonable-but-unconfirmed assumption.

## Suggested next step

The two firmware-relevant fixes — reading `0x0024` instead of `0x0025` for output frequency, and reading `0x0080` (with a corrected fault-name table) instead of `0x0021` for fault codes, plus fixing the reverse-detection bit from `0x0004` to `0x0002` — are all in `src/main.cpp`, not this reference file, and haven't been applied yet. Say the word if you want those done.
