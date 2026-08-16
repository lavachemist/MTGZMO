/* =============================================================================
   A1000 Modbus RTU Simulator — Raspberry Pi Pico (RP2040)
   Pretends to be a Yaskawa A1000 VFD on an RS-485 bus.

   Wiring (MAX485 breakout):
     GPIO 8  (Serial2 TX) → MAX485 DI  (RS-485 TX-I)
     GPIO 9  (Serial2 RX) → MAX485 RO  (RS-485 TX-O)
     GPIO 7               → MAX485 DE + RE tied together (HIGH = transmit)
     3.3 V                → MAX485 VCC
     GND                  → MAX485 GND
     A (+) / B (−) bus    → connect to master's A / B

   Supported Modbus function codes:
     FC 03  Read Holding Registers
     FC 06  Write Single Register

   Simulated registers — matches the real A1000's actual MEMOBUS/Modbus map
   (Yaskawa Technical Manual SIEP C710616 41H; see src/a1000_modbus.json and
   a1000_modbus_corrections.md for how this was verified — an earlier version
   of this simulator, and of main.cpp, used a fabricated map that didn't match
   the real drive):
     0x0001  Command word  (R/W) — bit 0 = fwd, bit 1 = rev, bit 3 = fault reset
     0x0002  Frequency reference (R/W) — 0.01 Hz units
     0x0020  Drive Status 1 (R) — bit0 During Run, bit1 During Reverse,
             bit2 Drive Ready, bit3 Fault, bit14 ComRef enabled, bit15 ComCtrl
             enabled
     0x0021  Fault Contents 1 (R) — bitmask, not a decodable code; simplified
             here to bit 0 set whenever any fault is active
     0x0024  Output Frequency (R) — tracks freq ref when running
     0x0080  Current Fault code (R) — parameter U2-01; this is the register
             that actually decodes to a fault name (Table C.6), unlike 0x0021

   All other registers return 0x0000 on read; writes to unknown registers
   return Modbus exception 02 (Illegal Data Address).
   ============================================================================= */

#include <Arduino.h>
#include "modbus_crc.h"

/* ---- RS-485 pins ---- */
#define SIM_TX_PIN   8      /* Serial2 TX → MAX485 DI  */
#define SIM_RX_PIN   9      /* Serial2 RX ← MAX485 RO  */
#define SIM_DE_PIN   7      /* MAX485 DE/RE — HIGH = transmit */

#define SIM_BAUD     9600
#define SIM_SLAVE    1      /* Simulated slave address — must match master config */

/* ---- Simulated drive state ---- */
static uint16_t sim_freq_ref    = 0;      /* last written frequency reference (0.01 Hz) */
static uint16_t sim_output_freq = 0;      /* tracks freq_ref when running */
static bool     sim_running     = false;
static bool     sim_reverse     = false;  /* current active direction */
static bool     sim_rev_pending = false;  /* direction requested while running — applied after ramp to 0 */
static bool     sim_dir_change  = false;  /* true when ramping to 0 before a direction flip */
static uint16_t sim_fault_code  = 0;      /* 0 = no fault */

/* ---- Helpers ---- */

static void send_frame(const uint8_t *frame, uint8_t len)
{
    digitalWrite(SIM_DE_PIN, HIGH);
    Serial2.write(frame, len);
    Serial2.flush();
    digitalWrite(SIM_DE_PIN, LOW);

    Serial.print("[SIM TX]");
    for (uint8_t i = 0; i < len; i++) {
        char tmp[4];
        snprintf(tmp, sizeof(tmp), " %02X", frame[i]);
        Serial.print(tmp);
    }
    Serial.println();
}

/* Send a Modbus exception response */
static void send_exception(uint8_t fc, uint8_t code)
{
    uint8_t frame[5];
    frame[0] = SIM_SLAVE;
    frame[1] = fc | 0x80;
    frame[2] = code;
    uint16_t crc = modbus_crc(frame, 3);
    frame[3] = (uint8_t)(crc & 0xFF);
    frame[4] = (uint8_t)(crc >> 8);
    send_frame(frame, 5);
}

/* ---- Register read/write ---- */

/* Returns false if the address is not implemented */
static bool read_register(uint16_t addr, uint16_t &value)
{
    switch (addr) {
        case 0x0001: {
            /* Command word — reflect current state back */
            uint16_t cmd = 0;
            if (sim_running && !sim_reverse) cmd = 0x0001;
            if (sim_running &&  sim_reverse) cmd = 0x0002;
            value = cmd;
            return true;
        }
        case 0x0002:
            value = sim_freq_ref;
            return true;
        case 0x0020: {
            /* Drive Status 1 */
            uint16_t status = 0;
            if (sim_running)      status |= (1 << 0);   /* During Run */
            if (sim_reverse)      status |= (1 << 1);   /* During Reverse */
            if (!sim_fault_code)  status |= (1 << 2);   /* Drive Ready (no active fault) */
            if (sim_fault_code)   status |= (1 << 3);   /* Fault */
            status |= (1 << 14);                        /* ComRef enabled */
            status |= (1 << 15);                        /* ComCtrl enabled */
            value = status;
            return true;
        }
        case 0x0021:
            /* Fault Contents 1 bitmask — simplified to "some fault active" */
            value = sim_fault_code ? 0x0001 : 0x0000;
            return true;
        case 0x0024:
            value = sim_output_freq;
            return true;
        case 0x0080:
            /* U2-01 Current Fault — the register that actually decodes to a
               fault name; see fault_and_alarm_codes.current_fault_code in
               a1000_modbus.json for the real code values. */
            value = sim_fault_code;
            return true;
        default:
            /* Valid-looking but unimplemented registers return 0 */
            if (addr <= 0x007F) {
                value = 0;
                return true;
            }
            return false;
    }
}

static bool write_register(uint16_t addr, uint16_t value)
{
    switch (addr) {
        case 0x0001: {
            /* Command word */
            bool fwd   = (value & 0x0001) != 0;
            bool rev   = (value & 0x0002) != 0;
            bool reset = (value & 0x0008) != 0;

            if (reset) {
                sim_fault_code  = 0;
                sim_running     = false;
                sim_dir_change  = false;
                sim_output_freq = 0;
            }
            if (fwd && !rev) {
                if (sim_running && sim_reverse) {
                    /* Direction change while running — ramp to 0 first */
                    sim_rev_pending = false;
                    sim_dir_change  = true;
                } else {
                    sim_running    = true;
                    sim_reverse    = false;
                    sim_dir_change = false;
                }
            } else if (rev && !fwd) {
                if (sim_running && !sim_reverse) {
                    /* Direction change while running — ramp to 0 first */
                    sim_rev_pending = true;
                    sim_dir_change  = true;
                } else {
                    sim_running    = true;
                    sim_reverse    = true;
                    sim_dir_change = false;
                }
            } else if (!fwd && !rev && !reset) {
                sim_running    = false;
                sim_dir_change = false;
            }
            return true;
        }
        case 0x0002:
            sim_freq_ref = value;
            return true;
        default:
            return false;
    }
}

/* ---- Frame processing ---- */

static void process_frame(const uint8_t *buf, uint8_t len)
{
    /* Minimum valid frame: 4 bytes (addr + FC + 2 CRC) */
    if (len < 4) return;

    Serial.print("[SIM RX]");
    for (uint8_t i = 0; i < len; i++) {
        char tmp[4];
        snprintf(tmp, sizeof(tmp), " %02X", buf[i]);
        Serial.print(tmp);
    }
    Serial.println();

    /* Verify CRC */
    uint16_t recv_crc = (uint16_t)buf[len - 1] << 8 | buf[len - 2];
    if (modbus_crc(buf, len - 2) != recv_crc) {
        Serial.println("[SIM] bad CRC — discarded");
        return;
    }

    /* Check slave address */
    if (buf[0] != SIM_SLAVE) return;   /* not addressed to us */

    uint8_t fc = buf[1];
    Serial.printf("[SIM] FC=0x%02X slave=%u\n", fc, buf[0]);

    /* ---- FC 03: Read Holding Registers ---- */
    if (fc == 0x03) {
        if (len < 8) { send_exception(fc, 0x03); return; }
        uint16_t start = (uint16_t)buf[2] << 8 | buf[3];
        uint16_t count = (uint16_t)buf[4] << 8 | buf[5];
        Serial.printf("[SIM] FC03 read reg=0x%04X cnt=%u\n", start, count);

        if (count == 0 || count > 16) { send_exception(fc, 0x03); return; }

        /* Pre-check all addresses before building response */
        uint16_t values[16];
        for (uint16_t i = 0; i < count; i++) {
            if (!read_register(start + i, values[i])) {
                send_exception(fc, 0x02);   /* illegal data address */
                return;
            }
        }

        /* Build response: [slave][03][byte_count][data...][CRC_LO][CRC_HI] */
        uint8_t resp_len = 3 + count * 2 + 2;
        uint8_t resp[40];
        resp[0] = SIM_SLAVE;
        resp[1] = 0x03;
        resp[2] = (uint8_t)(count * 2);
        for (uint16_t i = 0; i < count; i++) {
            resp[3 + i * 2]     = (uint8_t)(values[i] >> 8);
            resp[3 + i * 2 + 1] = (uint8_t)(values[i] & 0xFF);
        }
        uint16_t crc = modbus_crc(resp, resp_len - 2);
        resp[resp_len - 2] = (uint8_t)(crc & 0xFF);
        resp[resp_len - 1] = (uint8_t)(crc >> 8);
        send_frame(resp, resp_len);
        return;
    }

    /* ---- FC 06: Write Single Register ---- */
    if (fc == 0x06) {
        if (len < 8) { send_exception(fc, 0x03); return; }
        uint16_t addr  = (uint16_t)buf[2] << 8 | buf[3];
        uint16_t value = (uint16_t)buf[4] << 8 | buf[5];
        Serial.printf("[SIM] FC06 write reg=0x%04X val=0x%04X\n", addr, value);

        if (!write_register(addr, value)) {
            Serial.printf("[SIM] FC06 reg=0x%04X — illegal address, sending exception\n", addr);
            send_exception(fc, 0x02);
            return;
        }

        /* Success: echo the request unchanged */
        send_frame(buf, len);
        return;
    }

    /* ---- FC 08: Loopback diagnostic ---- */
    if (fc == 0x08) {
        if (len < 8) { send_exception(fc, 0x03); return; }
        uint16_t sub = (uint16_t)buf[2] << 8 | buf[3];
        if (sub == 0x0000) {
            /* Echo the entire request */
            send_frame(buf, len);
        } else {
            send_exception(fc, 0x01);   /* unsupported sub-function */
        }
        return;
    }

    /* Unsupported function code */
    send_exception(fc, 0x01);
}

/* ---- Receive state machine ---- */

/* Maximum Modbus RTU frame: 256 bytes.
   Inter-frame gap detection: at 9600 8-N-2 one char = 11 bits ≈ 1.15 ms.
   Modbus spec requires 3.5 char silence ≈ 4 ms to mark end of frame. */
#define RX_BUF_SIZE  64
#define FRAME_GAP_MS  5    /* ms of silence that marks end of a frame */

static uint8_t  rx_buf[RX_BUF_SIZE];
static uint8_t  rx_len   = 0;
static uint32_t last_rx_ms = 0;

static void receive_byte(uint8_t b)
{
    if (rx_len < RX_BUF_SIZE)
        rx_buf[rx_len++] = b;
    last_rx_ms = millis();
}

/* ---- Drive simulation: output frequency ramps toward freq_ref ---- */
/* Called every 50 ms.
   Accel: 3 s from 0 → 60 Hz  = 20 Hz/s  = 2000 units/s = 100 units/tick
   Decel: 2 s from 60 Hz → 0  = 30 Hz/s  = 3000 units/s = 150 units/tick */
#define RAMP_ACCEL 100   /* 0.01 Hz units per 50 ms tick — 20 Hz/s */
#define RAMP_DECEL 150   /* 0.01 Hz units per 50 ms tick — 30 Hz/s */

static void update_drive()
{
    if (!sim_running) {
        /* Commanded stop — decelerate to 0 */
        if (sim_output_freq > RAMP_DECEL)
            sim_output_freq -= RAMP_DECEL;
        else
            sim_output_freq = 0;
        return;
    }

    if (sim_dir_change) {
        /* Mid-direction-change: ramp output to 0 before flipping direction */
        if (sim_output_freq > RAMP_DECEL) {
            sim_output_freq -= RAMP_DECEL;
        } else {
            /* Reached 0 — flip direction and start ramping up */
            sim_output_freq = 0;
            sim_reverse     = sim_rev_pending;
            sim_dir_change  = false;
        }
        return;
    }

    /* Normal run — ramp toward freq_ref */
    if (sim_output_freq < sim_freq_ref) {
        uint16_t step = min((uint32_t)RAMP_ACCEL, (uint32_t)(sim_freq_ref - sim_output_freq));
        sim_output_freq += step;
    } else if (sim_output_freq > sim_freq_ref) {
        uint16_t step = min((uint32_t)RAMP_DECEL, (uint32_t)(sim_output_freq - sim_freq_ref));
        sim_output_freq -= step;
    }
}

/* ---- Arduino entry points ---- */

void setup()
{
    Serial.begin(115200);

    pinMode(SIM_DE_PIN, OUTPUT);
    digitalWrite(SIM_DE_PIN, LOW);   /* receive mode */

    Serial2.begin(SIM_BAUD, SERIAL_8N2);
    Serial.println("[SIM] A1000 simulator ready — 9600 8-N-2, slave addr " + String(SIM_SLAVE));

    /* Optional: LED on GPIO 25 as a heartbeat */
    pinMode(PIN_LED, OUTPUT);
}

void loop()
{
    /* Drain RX bytes into buffer */
    while (Serial2.available()) {
        receive_byte((uint8_t)Serial2.read());
    }

    /* Process a complete frame once the inter-frame gap has elapsed */
    uint32_t now = millis();
    if (rx_len > 0 && (now - last_rx_ms) >= FRAME_GAP_MS) {
        process_frame(rx_buf, rx_len);
        rx_len = 0;
    }

    /* Update simulated drive state every 50 ms */
    static uint32_t last_drive_ms = 0;
    if (now - last_drive_ms >= 50) {
        last_drive_ms = now;
        update_drive();
    }

    /* Heartbeat: LED blinks at 1 Hz when running, 0.25 Hz when stopped */
    static uint32_t last_led_ms = 0;
    static bool     led_state   = false;
    uint32_t blink_ms = sim_running ? 500 : 2000;
    if (now - last_led_ms >= blink_ms) {
        last_led_ms = now;
        led_state   = !led_state;
        digitalWrite(PIN_LED, led_state);
    }
}
