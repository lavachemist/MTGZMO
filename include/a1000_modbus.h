#pragma once
/* =============================================================================
   Yaskawa A1000 MEMOBUS/Modbus RTU definitions

   Single source of truth for register addresses, status/command bits, and
   fault/alarm/exception code tables shared by src/main.cpp (Modbus master)
   and src/simulator.cpp (Modbus slave simulator). Derived from, and kept in
   sync with, src/a1000_modbus.json — that file is the canonical
   human-readable reference (with page citations back to the source) and the
   place to update first if anything here needs correcting; see
   src/a1000_modbus_corrections.md for how the underlying data was verified
   against the official YASKAWA AC Drive – A1000 Technical Manual
   (SIEP C710616 41H).

   This header intentionally includes more than main.cpp/simulator.cpp
   currently use — the full parameter address table, the complete (trimmed)
   fault code tables, exception codes, etc. — so it's directly reusable in
   another Arduino project talking to an A1000 without re-deriving values
   from the manual again. Anything NOT used by this project's two .cpp files
   is still real, verified data; it's just not called from anywhere yet.

   All lookup tables here are declared `static` so this header can be
   #included from multiple translation units without ODR/linker conflicts
   (this project only ever compiles one of main.cpp/simulator.cpp into a
   given binary, but a project reusing this header elsewhere might not).
   ============================================================================= */

#include <stdint.h>
#include <stddef.h>

/* ==========================================================================
   Command / monitor registers — MEMOBUS/Modbus PDU addresses, used directly
   (no offset). Source: Technical Manual Table C.5, p.716-719.
   ========================================================================== */

#define A1000_REG_CMD_WORD          0x0001  /* R/W - run/stop/direction/fault-reset, Multi-Function Inputs 1-8 in bits 4-11 */
#define A1000_REG_FREQ_REF          0x0002  /* R/W - frequency reference, 0.01 Hz units by default (o1-03 dependent) */
#define A1000_REG_OUT_VOLT_BIAS     0x0003  /* R/W - output voltage gain/bias, 0.1% units, range 20-2000 */
#define A1000_REG_TORQUE_REF_LIMIT  0x0004  /* R/W - torque reference/limit, 0.1%, signed (torque control only) */
#define A1000_REG_TORQUE_COMP       0x0005  /* R/W - torque compensation, 0.1%, signed (torque control only) */
#define A1000_REG_PID_TARGET        0x0006  /* R/W - PID target, 0.01%, signed */
#define A1000_REG_ANALOG_OUT_FM     0x0007  /* R/W - analog output terminal FM setting, 10V/4000H */
#define A1000_REG_ANALOG_OUT_AM     0x0008  /* R/W - analog output terminal AM setting, 10V/4000H */
#define A1000_REG_MFDO_SETTINGS     0x0009  /* R/W - multi-function digital output settings, bitfield */
#define A1000_REG_PULSE_OUT_MP      0x000A  /* R/W - pulse output terminal MP setting, 1 Hz units, 0-32000 */
#define A1000_REG_CONTROL_SELECT    0x000F  /* R/W - control selection setting, bitfield */
#define A1000_REG_ANALOG_OUT_A3_1   0x001B  /* R/W - option AO-A3 analog output 1, 10V/4000H */
#define A1000_REG_ANALOG_OUT_A3_2   0x001C  /* R/W - option AO-A3 analog output 2, 10V/4000H */
#define A1000_REG_DIGITAL_OUT_A3    0x001D  /* R/W - option DO-A3 digital output (binary) */

#define A1000_REG_STATUS_1          0x0020  /* R   - Drive Status 1 — see A1000_STATUS1_* bit masks below */
#define A1000_REG_FAULT_BITMASK_1   0x0021  /* R   - Fault Contents 1 — CATEGORY bitmask, NOT a single fault code */
#define A1000_REG_DATA_LINK_STATUS  0x0022  /* R   - Data Link Status, bitfield (EEPROM write / limit-error flags) */
#define A1000_REG_FREQ_REF_MONITOR  0x0023  /* R   - Frequency Reference monitor value */
#define A1000_REG_OUTPUT_FREQ       0x0024  /* R   - Output Frequency — the real one; see A1000_REG_OUTPUT_VOLTAGE */
#define A1000_REG_OUTPUT_VOLTAGE    0x0025  /* R   - Output Voltage Reference, 0.1V (or 1V if H5-10=1) — NOT frequency */
#define A1000_REG_OUTPUT_CURRENT    0x0026  /* R   - Output Current, units vary by drive model (0.01/0.1/1 A) */
#define A1000_REG_OUTPUT_POWER      0x0027  /* R   - Output Power (decimal precision not stated in manual excerpt) */
#define A1000_REG_TORQUE_REF        0x0028  /* R   - Torque Reference — NOT DC bus voltage */
#define A1000_REG_FAULT_BITMASK_2   0x0029  /* R   - Fault Contents 2 — bitmask */
#define A1000_REG_ALARM_1           0x002A  /* R   - Alarm Contents 1 — bitmask */
#define A1000_REG_INPUT_TERM_STATUS 0x002B  /* R   - Input Terminal Status, bitfield (S1-S8 closed) */
#define A1000_REG_STATUS_2          0x002C  /* R   - Drive Status 2 — see A1000_STATUS2_* bit masks below */
#define A1000_REG_OUTPUT_TERM_STAT  0x002D  /* R   - Output Terminal Status, bitfield */
#define A1000_REG_FREQ_REF_BIAS     0x002F  /* R   - Frequency Reference Bias (Up/Down2 function), 0.1% */
#define A1000_REG_DC_BUS_VOLTAGE    0x0031  /* R   - DC Bus Voltage, 1 Vdc (verified) */
#define A1000_REG_TORQUE_REF_U109   0x0032  /* R   - Torque Reference (U1-09), 1% */
#define A1000_REG_PRODUCT_CODE_1    0x0034  /* R   - ASCII product type (A0 = A1000) */
#define A1000_REG_PRODUCT_CODE_2    0x0035  /* R   - ASCII region code */
#define A1000_REG_PID_FEEDBACK      0x0038  /* R   - PID feedback, 0.1%, unsigned, 100%/max output freq */
#define A1000_REG_PID_INPUT         0x0039  /* R   - PID input (error), 0.1%, signed */
#define A1000_REG_PID_OUTPUT        0x003A  /* R   - PID output, 0.1%, signed */

#define A1000_REG_MINOR_FAULT_CODE  0x007F  /* R   - Minor Fault Code — decodes via a1000_minor_fault_name() */
#define A1000_REG_CURRENT_FAULT     0x0080  /* R   - U2-01 Current Fault — the real decodable fault code (Table C.6) */
#define A1000_REG_PREVIOUS_FAULT    0x0081  /* R   - U2-02 Previous Fault, same numbering as Current Fault */

/* ==========================================================================
   Communication setup parameters (H5-xx), accessed via the parameter
   address space. Only H5-01 through H5-07 have confirmed addresses (all
   from Technical Manual C.4, p.707-708); H5-09/10/11/12/17/18 are
   documented in a1000_modbus.json with their settings/behavior but their
   MEMOBUS addresses were not looked up — do not guess them.
   ========================================================================== */

#define A1000_PARAM_H5_01_SLAVE_ADDR       0x0320  /* range 0-FF hex, default 1F (31 dec) */
#define A1000_PARAM_H5_02_BAUD_RATE        0x0321  /* range 0-8: 1200,2400,4800,9600,19200,38400,57600,76800,115200; default 3 (9600) */
#define A1000_PARAM_H5_03_PARITY           0x0322  /* range 0-2: none/even/odd; default 0. Parity ONLY, not stop bits. */
#define A1000_PARAM_H5_04_STOP_METHOD_CE   0x0323  /* range 0-3: ramp/coast/fast-stop/alarm-only; default 3 */
#define A1000_PARAM_H5_05_FAULT_DETECT     0x0324  /* range 0-1: disabled/enabled; default 1 */
#define A1000_PARAM_H5_06_TX_WAIT_MS       0x0325  /* range 5-65 ms; default 5 */
#define A1000_PARAM_H5_07_RTS_CONTROL      0x0326  /* range 0-1: always-on/RTS-controlled; default 1 */

/* ==========================================================================
   Drive Status 1 (register A1000_REG_STATUS_1 / 0x0020) bit masks
   ========================================================================== */

#define A1000_STATUS1_DURING_RUN       (1u << 0)
#define A1000_STATUS1_DURING_REVERSE   (1u << 1)
#define A1000_STATUS1_DRIVE_READY      (1u << 2)
#define A1000_STATUS1_FAULT            (1u << 3)
#define A1000_STATUS1_DATA_SET_ERROR   (1u << 4)
#define A1000_STATUS1_MFO_1            (1u << 5)   /* Multi-Function Contact Output 1, terminal M1-M2 */
#define A1000_STATUS1_MFO_2            (1u << 6)   /* Multi-Function Contact Output 2, terminal M3-M4 */
#define A1000_STATUS1_MFO_3            (1u << 7)   /* Multi-Function Contact Output 3, terminal M5-M6 */
#define A1000_STATUS1_COMREF_ENABLED   (1u << 14)
#define A1000_STATUS1_COMCTRL_ENABLED  (1u << 15)

/* ==========================================================================
   Drive Status 2 (register A1000_REG_STATUS_2 / 0x002C) bit masks
   ========================================================================== */

#define A1000_STATUS2_DURING_RUN           (1u << 0)
#define A1000_STATUS2_ZERO_SPEED           (1u << 1)
#define A1000_STATUS2_SPEED_AGREE          (1u << 2)
#define A1000_STATUS2_USER_SPEED_AGREE     (1u << 3)
#define A1000_STATUS2_FREQ_DETECTION_1     (1u << 4)
#define A1000_STATUS2_FREQ_DETECTION_2     (1u << 5)
#define A1000_STATUS2_DRIVE_READY          (1u << 6)
#define A1000_STATUS2_DURING_UNDERVOLTAGE  (1u << 7)
#define A1000_STATUS2_DURING_BASEBLOCK     (1u << 8)
#define A1000_STATUS2_FREQ_REF_FROM_KEYPAD (1u << 9)
#define A1000_STATUS2_RUN_CMD_FROM_KEYPAD  (1u << 10)
#define A1000_STATUS2_OVER_UNDERTORQUE     (1u << 11)
#define A1000_STATUS2_FREQ_REF_LOSS        (1u << 12)
#define A1000_STATUS2_DURING_FAULT_RESTART (1u << 13)
#define A1000_STATUS2_FAULT                (1u << 14)
#define A1000_STATUS2_COMM_TIMEOUT         (1u << 15)

/* ==========================================================================
   Command word (register A1000_REG_CMD_WORD / 0x0001) bit masks and the
   common pre-built command values this project actually sends.
   ========================================================================== */

#define A1000_CMD_FORWARD_RUN    (1u << 0)  /* H5-12=0 (default): forward run/stop. H5-12=1: generic run/stop. */
#define A1000_CMD_REVERSE_RUN    (1u << 1)  /* H5-12=0 (default): reverse run/stop. H5-12=1: direction select. */
#define A1000_CMD_EXT_FAULT_EF0  (1u << 2)
#define A1000_CMD_FAULT_RESET    (1u << 3)  /* rising edge resets an active fault; clear back to 0 after */
#define A1000_CMD_MFI_1          (1u << 4)  /* Multi-Function Input 1 (H1-01) */
#define A1000_CMD_MFI_2          (1u << 5)  /* Multi-Function Input 2 (H1-02) */
#define A1000_CMD_MFI_3          (1u << 6)  /* Multi-Function Input 3 (H1-03) */
#define A1000_CMD_MFI_4          (1u << 7)  /* Multi-Function Input 4 (H1-04) */
#define A1000_CMD_MFI_5          (1u << 8)  /* Multi-Function Input 5 (H1-05) */
#define A1000_CMD_MFI_6          (1u << 9)  /* Multi-Function Input 6 (H1-06) */
#define A1000_CMD_MFI_7          (1u << 10) /* Multi-Function Input 7 (H1-07) */
#define A1000_CMD_MFI_8          (1u << 11) /* Multi-Function Input 8 (H1-08) */
/* bits 12-15 reserved */

#define A1000_CMD_WORD_STOP          0x0000
#define A1000_CMD_WORD_RUN_FORWARD   0x0001
#define A1000_CMD_WORD_RUN_REVERSE   0x0002

/* ==========================================================================
   Modbus function codes this project uses/expects, and A1000-specific
   exception codes (NOT the generic Modbus exception list — the A1000 uses
   its own set). Source: Technical Manual C.11, p.733.
   ========================================================================== */

#define A1000_FC_READ_HOLDING_REGS    0x03
#define A1000_FC_WRITE_SINGLE_REG     0x06
#define A1000_FC_LOOPBACK_TEST        0x08
#define A1000_FC_WRITE_MULTIPLE_REGS  0x10

struct A1000ExceptionInfo { uint8_t code; const char *name; };

static const A1000ExceptionInfo A1000_EXCEPTION_CODES[] = {
    { 0x01, "Function Code Error" },
    { 0x02, "Register Number Error" },
    { 0x03, "Bit Count Error" },
    { 0x21, "Data Setting Error" },
    { 0x22, "Write Mode Error" },
    { 0x23, "DC Bus Undervoltage Write Error" },
    { 0x24, "Write Error During Parameter Process" },
    { 0x25, "Writing into EEPROM Disabled" },
};
#define A1000_EXCEPTION_CODES_COUNT (sizeof(A1000_EXCEPTION_CODES) / sizeof(A1000_EXCEPTION_CODES[0]))

static inline const char *a1000_exception_name(uint8_t code)
{
    for (size_t i = 0; i < A1000_EXCEPTION_CODES_COUNT; i++)
        if (A1000_EXCEPTION_CODES[i].code == code) return A1000_EXCEPTION_CODES[i].name;
    return "Unknown exception code";
}

/* ==========================================================================
   Fault Contents 1 / 2 and Alarm Contents 1 — CATEGORY bitmasks (registers
   0x0021, 0x0029, 0x002A). Each bit covers a group of related faults, not a
   single fault; use A1000_REG_CURRENT_FAULT / A1000_REG_MINOR_FAULT_CODE
   below to get an actual decodable single fault code.
   ========================================================================== */

static const char *const A1000_FAULT_BITMASK_1_NAMES[16] = {
    /* 0  */ "Overcurrent (oC), Ground fault (GF)",
    /* 1  */ "Drive Overheat Warning (ov)",
    /* 2  */ "Drive Overload (oL2)",
    /* 3  */ "Overheat 1 (oH1), Drive Overheat Warning (oH2)",
    /* 4  */ "Dynamic Braking Transistor Fault (rr), Braking Resistor Overheat (rH)",
    /* 5  */ "Reserved",
    /* 6  */ "PID Feedback Loss (FbL / FbH)",
    /* 7  */ "EF to EF8: External Fault",
    /* 8  */ "CPFoo: Hardware Fault (includes oFx)",
    /* 9  */ "Motor Overload (oL1), Overtorque Detection 1/2 (oL3/oL4), Undertorque Detection 1/2 (UL3/UL4)",
    /* 10 */ "PG Disconnected (PGo), PG Hardware Fault (PGoH), Overspeed (oS), Speed Deviation (dEv)",
    /* 11 */ "Main Circuit Undervoltage (Uv)",
    /* 12 */ "DC Bus Undervoltage (Uv1), Control Power Supply Voltage Fault (Uv2), Undervoltage 3 (Uv3)",
    /* 13 */ "Output Phase Loss (LF), Input Phase Loss (PF)",
    /* 14 */ "MEMOBUS/Modbus Communication Error (CE), Option Communication Error (bUS)",
    /* 15 */ "External Digital Operator Connection Fault (oPr)",
};

static const char *const A1000_FAULT_BITMASK_2_NAMES[16] = {
    /* 0  */ "Output Short Circuit or IGBT Fault (SC)",
    /* 1  */ "Ground Fault (GF)",
    /* 2  */ "Input Phase Loss (PF)",
    /* 3  */ "Output Phase Loss (LF)",
    /* 4  */ "Braking Resistor Overheat (rH)",
    /* 5  */ "Reserved",
    /* 6  */ "Motor Overheat 2 / PTC input (oH4)",
    /* 7  */ "Reserved", /* 8 */ "Reserved", /* 9 */ "Reserved", /* 10 */ "Reserved",
    /* 11 */ "Reserved", /* 12 */ "Reserved", /* 13 */ "Reserved", /* 14 */ "Reserved", /* 15 */ "Reserved",
};

static const char *const A1000_ALARM_1_NAMES[16] = {
    /* 0  */ "Reserved",
    /* 1  */ "Reserved",
    /* 2  */ "Forward/Reverse Run Command Input Error (EF)",
    /* 3  */ "Drive Baseblock (bb)",
    /* 4  */ "Overtorque Detection 1 (oL3)",
    /* 5  */ "Heatsink Overheat (oH)",
    /* 6  */ "Drive Overheat Warning (ov)",
    /* 7  */ "Undervoltage (Uv)",
    /* 8  */ "Internal Fan Fault (FAn)",
    /* 9  */ "MEMOBUS/Modbus Communication Error (CE)",
    /* 10 */ "Option Communication Error (bUS)",
    /* 11 */ "Undertorque Detection 1/2 (UL3/UL4)",
    /* 12 */ "Motor Overheat (oH3)",
    /* 13 */ "PID Feedback Loss (FbL, FbH)",
    /* 14 */ "Reserved",
    /* 15 */ "Serial Communication Transmission Error (CALL)",
};

/* ==========================================================================
   Current Fault (register A1000_REG_CURRENT_FAULT / 0x0080, parameter
   U2-01) — the register that actually decodes to a single fault name.
   0x0081 (U2-02, Previous Fault) uses the same numbering.
   Source: Technical Manual Table C.6, p.729-730. Trimmed to the codes most
   relevant to this project (oFx option-card sub-codes, dv/CPF hardware
   sub-codes, and MECHATROLINK-specific codes omitted — see the Technical
   Manual for the full ~90-entry list, including CPF00-CPF45 at
   0x0083-0x009F+).
   ========================================================================== */

struct A1000FaultCode { uint16_t code; const char *name; };

static const A1000FaultCode A1000_CURRENT_FAULT_CODES[] = {
    { 0x0002, "Uv1 - DC bus undervoltage" },
    { 0x0003, "Uv2 - Control power supply undervoltage" },
    { 0x0004, "Uv3 - Soft-charge fault" },
    { 0x0005, "SC - Output short-circuit/IGBT fault" },
    { 0x0006, "GF - Ground fault" },
    { 0x0007, "oC - Overcurrent" },
    { 0x0008, "ov - Overvoltage" },
    { 0x0009, "oH - Heatsink overheat" },
    { 0x000A, "oH1 - Heatsink overheat 1" },
    { 0x000B, "oL1 - Motor overload" },
    { 0x000C, "oL2 - Drive overload" },
    { 0x000D, "oL3 - Overtorque 1" },
    { 0x000E, "oL4 - Overtorque 2" },
    { 0x000F, "rr - DB transistor fault" },
    { 0x0010, "rH - DB resistor overheat" },
    { 0x0011, "EF3 - External fault, terminal S3" },
    { 0x0012, "EF4 - External fault, terminal S4" },
    { 0x0013, "EF5 - External fault, terminal S5" },
    { 0x0014, "EF6 - External fault, terminal S6" },
    { 0x0015, "EF7 - External fault, terminal S7" },
    { 0x0016, "EF8 - External fault, terminal S8" },
    { 0x0017, "FAn - Internal fan fault" },
    { 0x0018, "oS - Overspeed" },
    { 0x0019, "dEv - Speed deviation" },
    { 0x001A, "PGo - PG disconnect" },
    { 0x001B, "PF - Input phase loss" },
    { 0x001C, "LF - Output phase loss" },
    { 0x001D, "oH3 - Motor overheat (PTC)" },
    { 0x001E, "oPr - Digital operator connection fault" },
    { 0x001F, "Err - EEPROM write error" },
    { 0x0020, "oH4 - Motor overheat (PTC)" },
    { 0x0021, "CE - Modbus communication error" },
    { 0x0022, "bUS - Option communication error" },
    { 0x0025, "CF - Control fault" },
    { 0x0026, "SvE - Zero-servo fault" },
    { 0x0027, "EF0 - Option external fault" },
    { 0x0028, "FbL - PID feedback loss" },
    { 0x0029, "UL3 - Undertorque detection 1" },
    { 0x002A, "UL4 - Undertorque detection 2" },
    { 0x002B, "oL7 - High slip braking overload" },
    { 0x0030, "Hardware fault (incl. oFx)" },
    { 0x0050, "oH5 - Motor overheat (NTC)" },
    { 0x0051, "LSo - Low speed fault" },
    { 0x0052, "nSE - Node setup fault" },
    { 0x0053, "THo - Thermistor disconnect" },
};
#define A1000_CURRENT_FAULT_CODES_COUNT (sizeof(A1000_CURRENT_FAULT_CODES) / sizeof(A1000_CURRENT_FAULT_CODES[0]))

static inline const char *a1000_current_fault_name(uint16_t code)
{
    for (size_t i = 0; i < A1000_CURRENT_FAULT_CODES_COUNT; i++)
        if (A1000_CURRENT_FAULT_CODES[i].code == code) return A1000_CURRENT_FAULT_CODES[i].name;
    return "Unknown/uncommon fault";
}

/* ==========================================================================
   Minor Fault Code (register A1000_REG_MINOR_FAULT_CODE / 0x007F).
   A SEPARATE numbering scheme from Current Fault above — do not mix them up.
   Source: Technical Manual Table C.7, p.731. Same trimming rationale as
   A1000_CURRENT_FAULT_CODES.
   ========================================================================== */

static const A1000FaultCode A1000_MINOR_FAULT_CODES[] = {
    { 0x0001, "Uv - Undervoltage" },
    { 0x0002, "ov - Drive overheat warning" },
    { 0x0003, "oH - Heatsink overheat" },
    { 0x0004, "oH2 - Drive overheat warning" },
    { 0x0005, "oL3 - Overtorque 1" },
    { 0x0006, "oL4 - Overtorque 2" },
    { 0x0007, "EF - Forward/Reverse run command input error" },
    { 0x0008, "bb - Drive baseblock" },
    { 0x0009, "EF3 - External fault, terminal S3" },
    { 0x000A, "EF4 - External fault, terminal S4" },
    { 0x000B, "EF5 - External fault, terminal S5" },
    { 0x000C, "EF6 - External fault, terminal S6" },
    { 0x000D, "EF7 - External fault, terminal S7" },
    { 0x000E, "EF8 - External fault, terminal S8" },
    { 0x000F, "FAn - Internal fan fault" },
    { 0x0010, "oS - Overspeed" },
    { 0x0011, "dEv - Speed deviation" },
    { 0x0012, "PGo - PG disconnected" },
    { 0x0014, "CE - Modbus communication error" },
    { 0x0015, "bUS - Option communication error" },
    { 0x0016, "CALL - Serial communication transmission error" },
    { 0x0017, "oL1 - Motor overload" },
    { 0x0018, "oL2 - Drive overload" },
    { 0x001A, "EF0 - Option card external fault" },
    { 0x001B, "rUn - Motor switch command input during run" },
    { 0x001D, "CALL - Serial communication transmission error" },
    { 0x001E, "UL3 - Undertorque detection 1" },
    { 0x001F, "UL4 - Undertorque detection 2" },
    { 0x0020, "SE - Modbus communication test mode error" },
    { 0x0022, "oH3 - Motor overheat" },
    { 0x0027, "FbL - PID feedback loss" },
    { 0x0028, "FbH - PID feedback loss" },
    { 0x002A, "dnE - Drive disabled" },
    { 0x002B, "PGo - PG disconnected" },
    { 0x0034, "HCA - High current alarm" },
    { 0x0035, "LT-1 - Cooling fan maintenance time" },
    { 0x0036, "LT-2 - Capacitor maintenance time" },
    { 0x0039, "EF1 - External fault, terminal S1" },
    { 0x003A, "EF2 - External fault, terminal S2" },
    { 0x0041, "voF - Output voltage detection fault" },
    { 0x0042, "TrPC - IGBT maintenance time 90%" },
    { 0x0043, "LT-3 - Soft charge bypass relay maintenance time" },
    { 0x0044, "LT-4 - IGBT maintenance time 50%" },
    { 0x0045, "boL - Braking transistor overload" },
    { 0x0048, "oH5 - Motor overheat (NTC)" },
    { 0x004D, "THo - Thermistor disconnect" },
};
#define A1000_MINOR_FAULT_CODES_COUNT (sizeof(A1000_MINOR_FAULT_CODES) / sizeof(A1000_MINOR_FAULT_CODES[0]))

static inline const char *a1000_minor_fault_name(uint16_t code)
{
    for (size_t i = 0; i < A1000_MINOR_FAULT_CODES_COUNT; i++)
        if (A1000_MINOR_FAULT_CODES[i].code == code) return A1000_MINOR_FAULT_CODES[i].name;
    return "Unknown/uncommon alarm";
}

/* ==========================================================================
   Parameter registers — drive setup parameters (as opposed to the runtime
   command/monitor registers above), accessed via the parameter address
   space with the same FC03/FC06/FC10. Every address below was individually
   confirmed against Technical Manual Appendix B (B.3-B.7, p.559-590).
   Nothing in this project currently reads/writes these at runtime; they're
   included for future use (parameter backup/restore tooling, a setup
   wizard, etc.) rather than fabricated ahead of need.
   ========================================================================== */

struct A1000ParamInfo { uint16_t address; const char *param; const char *name; };

static const A1000ParamInfo A1000_PARAMETERS[] = {
    { 0x0102, "A1-02", "Control Method Selection" },
    { 0x0103, "A1-03", "Initialize Parameters" },
    { 0x0180, "b1-01", "Frequency Reference Selection 1" },
    { 0x0181, "b1-02", "Run Command Selection 1" },
    { 0x0183, "b1-04", "Reverse Operation Selection" },
    { 0x0200, "C1-01", "Acceleration Time 1" },
    { 0x0201, "C1-02", "Deceleration Time 1" },
    { 0x0202, "C1-03", "Acceleration Time 2" },
    { 0x0203, "C1-04", "Deceleration Time 2" },
    { 0x0280, "d1-01", "Frequency Reference 1" },
    { 0x0281, "d1-02", "Frequency Reference 2" },
    { 0x0282, "d1-03", "Frequency Reference 3" },
    { 0x0283, "d1-04", "Frequency Reference 4" },
    { 0x0289, "d2-01", "Frequency Reference Upper Limit" },
    { 0x028A, "d2-02", "Frequency Reference Lower Limit" },
    { 0x0300, "E1-01", "Input Voltage Setting" },
    { 0x0303, "E1-04", "Maximum Output Frequency" },
    { 0x0304, "E1-05", "Maximum Voltage" },
    { 0x0308, "E1-09", "Minimum Output Frequency" },
    { 0x030E, "E2-01", "Motor Rated Current" },
    { 0x030F, "E2-02", "Motor Rated Slip" },
    { 0x0310, "E2-03", "Motor No-Load Current" },
    { 0x0311, "E2-04", "Number of Motor Poles" },
    { 0x0312, "E2-05", "Motor Line-to-Line Resistance" },
};
#define A1000_PARAMETERS_COUNT (sizeof(A1000_PARAMETERS) / sizeof(A1000_PARAMETERS[0]))

/* Returns nullptr if addr isn't in the table above. */
static inline const A1000ParamInfo *a1000_lookup_parameter(uint16_t addr)
{
    for (size_t i = 0; i < A1000_PARAMETERS_COUNT; i++)
        if (A1000_PARAMETERS[i].address == addr) return &A1000_PARAMETERS[i];
    return nullptr;
}

/* ==========================================================================
   Scaling conventions. All are the DEFAULT scaling — frequency/speed values
   in particular depend on parameter o1-03 (see a1000_modbus.json notes) and
   should not be assumed unconditionally in a new integration.
   ========================================================================== */

#define A1000_SCALE_FREQ_HZ_PER_COUNT        0.01f   /* o1-03 dependent; this is the o1-03=0 (default) case */
#define A1000_SCALE_CURRENT_A_PER_COUNT      0.1f    /* model-dependent — see A1000_REG_OUTPUT_CURRENT comment */
#define A1000_SCALE_VOLTAGE_V_PER_COUNT      0.1f    /* H5-10 dependent for register 0x0025 specifically */
#define A1000_SCALE_DC_BUS_VOLTAGE_V_PER_COUNT 1.0f  /* verified: register 0x0031 is whole volts */
#define A1000_SCALE_TORQUE_PCT_PER_COUNT     0.1f
#define A1000_SCALE_ACCEL_DECEL_S_PER_COUNT  0.1f    /* verified */
#define A1000_SCALE_RESISTANCE_OHM_PER_COUNT 0.001f
