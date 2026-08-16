#include <Arduino.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <math.h>
#include <SPI.h>
/* ---- Stepper driver (TMC5160 via TMCStepper library) ----
   To swap drivers: replace this include, the defines, the driver object,
   stepper_init(), and stepper_set_rpm() below. Nothing else needs to change. */
#include <TMCStepper.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include "modbus_crc.h"
#include "a1000_modbus.h"

/* ---- Display size (must match lv_conf.h) ---- */
#define DISP_HOR_RES 240
#define DISP_VER_RES 240

/* ---- Encoder ---- */
#define ENCODER_A     2
#define ENCODER_B     3
static volatile uint32_t encoder_ppr  = 600;   /* Pulses per revolution — runtime adjustable */
static volatile uint32_t hob_threads    = 1;   /* Stepper RPM units — ratio numerator   */
static volatile uint32_t gear_teeth     = 32;  /* Encoder RPM units — ratio denominator */
static volatile uint32_t pulley_driver  = 1;   /* Teeth on stepper-side pulley           */
static volatile uint32_t pulley_driven  = 1;   /* Teeth on output-side pulley            */
#define RPM_UPDATE_MS 50            /* Recalculate RPM every 50 ms */
#define METER_MAX_RPM 4000

/* ---- Stepper pin / parameter defines (TMC5160-specific) ---- */
#define TMC_MOSI        11          /* SPI1 TX  */
#define TMC_MISO        12          /* SPI1 RX  */
#define TMC_SCK         10          /* SPI1 SCK */
#define TMC_CS          13          /* Chip select */
#define TMC_EN          15          /* Enable — LOW = enabled, HIGH = disabled */
#define TMC_R_SENSE     0.075f      /* Sense resistor value in ohms */
#define TMC_RMS_CURRENT 1000        /* Motor RMS current in mA */

#define BTN_PIN            14       /* IP display button — pulled up, active LOW */
#define VFD_BTN_START      4        /* VFD Start (run forward) — pulled up, active LOW */
#define VFD_BTN_REVERSE    5        /* VFD Reverse (run reverse) — pulled up, active LOW */
#define VFD_BTN_STOP       6        /* VFD Stop — pulled up, active LOW */
#define RPM_SRC_BTN       16        /* RPM source select — held LOW = VFD, open = encoder */
#define VFD_LOCK_BTN      22        /* Web VFD control lockout jumper — must be present (LOW)
                                        to unlock; absent/open (HIGH) fails safe to locked */
#define VFD_POT_PIN       26        /* RPM potentiometer wiper — ADC0 (GPIO 26) */
#define VFD_POT_DEADBAND  32        /* ADC counts of change required to trigger a write (~0.5 Hz hysteresis) */
#define VFD_POT_MIN       100       /* ADC counts at full CCW — maps to 0 Hz (tune if needed) */
#define MOTOR_STEPS     200         /* Full steps per revolution */
#define MICROSTEPS      256         /* Microstep resolution */

/* ---- RS-485 / VFD (UART1) ---- */
#define VFD_TX_PIN      8           /* UART1 TX → RS-485 RX-I */
#define VFD_RX_PIN      9           /* UART1 RX ← RS-485 TX-O */
#define VFD_DE_PIN      7           /* RS-485 DE/RE — HIGH = transmit, LOW = receive */
#define VFD_BAUD        9600
#define VFD_POLL_MS     500         /* Status poll interval */
#define VFD_DIR_SYNC_GRACE_MS  3000 /* After we command a direction, ignore the drive's
                                       reported direction bit for this long — the drive
                                       ramps to 0 before actually reversing, so polling
                                       mid-ramp would read the old direction and clobber
                                       our just-issued command, causing the arrow to blink
                                       back to the old direction before settling. Tune this
                                       to be longer than your VFD's decel+accel ramp time. */

static uint8_t  vfd_slave      = 1;     /* Modbus slave address — runtime adjustable */
static uint16_t vfd_max_hz     = 6000;  /* Max frequency setpoint (0.01 Hz units, 6000 = 60.00 Hz) */
static uint16_t vfd_base_hz    = 6000;  /* Baseline frequency (0.01 Hz units) for RPM scaling */
static uint16_t vfd_base_rpm   = 1750;  /* Motor RPM at baseline frequency */

/* Live VFD state updated by vfd_poll() */
static volatile uint16_t vfd_status_word  = 0;
static volatile uint16_t vfd_fault_code   = 0;
static volatile bool     vfd_running      = false;
static volatile uint16_t vfd_freq_ref     = 0;   /* last written frequency reference */
static volatile uint16_t vfd_output_freq  = 0;   /* actual output frequency (reg 0x0024) */
static volatile bool     vfd_comms_ok     = false;

/* When false, raw TX/RX byte dumps and FC03 OK lines are suppressed.
   Set true around explicit commands; left false during background polls. */
static bool vfd_log_raw = false;

/* Tracks the last commanded direction — shared between physical buttons and web handlers.
   false = forward, true = reverse. Start/Stop always uses this; Reverse flips it. */
static bool vfd_reverse_active = false;
static uint32_t vfd_dir_cmd_ms = 0;  /* millis() timestamp of the last local direction command */

/* ---- WiFi ---- */
#define WIFI_AP_SSID  "MTGizmo"
#define WIFI_AP_PASS  "hobbing1"        /* min 8 chars for WPA2 */
#define WIFI_HOSTNAME "gizmo.mt"        /* DNS name in AP mode */

enum WifiMode { MODE_AP, MODE_STA };
static WifiMode   wifi_mode              = MODE_AP;  /* saved/configured mode (persisted, drives retry-on-reboot) */
static WifiMode   current_wifi_mode      = MODE_AP;  /* actual live radio mode, for UI display */
static String     sta_ssid               = "";
static String     sta_password           = "";
static bool       wifi_reconfig_pending  = false;  /* set by web handler, applied in loop() */
static String     current_ip   = "192.168.4.1";
static DNSServer  dnsServer;

/* output_rpm  = encoder_rpm × (hob_threads / gear_teeth)
   stepper_rpm = output_rpm  × (pulley_driven / pulley_driver)
   combined:  stepper_rpm = encoder_rpm × (hob_threads / gear_teeth) × (pulley_driven / pulley_driver) */
static volatile bool  encoder_reversed  = false;  /* flip encoder direction via web UI */

static TMC5160Stepper  driver(TMC_CS, TMC_R_SENSE, TMC_MOSI, TMC_MISO, TMC_SCK);
static WebServer       server(80);

/* Signed position counter — incremented or decremented in ISR based on direction */
static volatile int32_t encoder_pos = 0;

void encoderA_isr()
{
    /* Channel B HIGH when A rises → forward; LOW → reverse.
       encoder_reversed flips the sense without rewiring. */
    bool forward = (digitalRead(ENCODER_B) != encoder_reversed);
    if (forward)
        encoder_pos++;
    else
        encoder_pos--;
}

/* ---- Draw buffer ---- */
#define DRAW_BUF_ROWS 10
static lv_color_t    draw_buf_1[DISP_HOR_RES * DRAW_BUF_ROWS];

static TFT_eSPI      tft;

/* LVGL widgets updated each RPM cycle */
static lv_obj_t              *meter;
static lv_meter_indicator_t  *needle;
static lv_obj_t              *rpm_label;
static lv_obj_t              *rpm_readout;
static lv_obj_t              *arrow_fwd;      /* right-arrow: VFD forward indicator  */
static lv_obj_t              *arrow_rev;      /* left-arrow:  VFD reverse indicator  */
static lv_obj_t              *ip_overlay;     /* IP address pop-up label */

/* ---- LVGL flush callback ---- */
static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)color_p, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(drv);
}

/* ---- Build the tachometer UI ---- */
static void create_tachometer(void)
{
    /* Dark charcoal screen background */
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x1A1A1A), LV_PART_MAIN);

    meter = lv_meter_create(lv_scr_act());
    lv_obj_set_size(meter, 240, 240);
    lv_obj_center(meter);

    /* Dark face, no visible border, no padding so ticks reach the edge */
    lv_obj_set_style_bg_color(meter, lv_color_hex(0x2B2B2B), LV_PART_MAIN);
    lv_obj_set_style_border_width(meter, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(meter, 0, LV_PART_MAIN);

    /* Scale: 0–400 internally (labels show 0–4 = ×1000 RPM), 240° sweep.
       100 internal units = 1000 RPM, giving smooth per-100-RPM needle steps. */
    lv_meter_scale_t *scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_range(meter, scale, 0, 400, 240, 150);

    /* Minor ticks: 41 total (every 10 units = 100 RPM) — white, 4px long */
    lv_meter_set_scale_ticks(meter, scale, 41, 2, 4, lv_color_hex(0xDDDDDD));

    /* Major ticks every 1000 RPM (every 10th minor) — 50% wider (5px), 14px long */
    lv_meter_set_scale_major_ticks(meter, scale, 10, 5, 14,
                                   lv_color_hex(0xFFFFFF), 0);

    /* Hide auto-generated tick labels by making them transparent */
    lv_obj_set_style_text_opa(meter, LV_OPA_TRANSP, LV_PART_TICKS);

    /* Mid ticks at every 500 RPM: second scale with 9 ticks across 0–400,
       every tick is a "major" (step=1) drawn at 8px long, 2px wide — white.
       9 ticks = 8 intervals of 50 units = 50,100,150,200,250,300,350,400.
       The ones that fall on 1000 RPM boundaries overlap the main scale ticks
       which is fine — they are the same colour and will just reinforce them. */
    lv_meter_scale_t *scale_mid = lv_meter_add_scale(meter);
    lv_meter_set_scale_range(meter, scale_mid, 0, 400, 240, 150);
    lv_meter_set_scale_ticks(meter, scale_mid, 9, 2, 14, lv_color_hex(0xDDDDDD));
    lv_meter_set_scale_major_ticks(meter, scale_mid, 1, 2, 14,
                                   lv_color_hex(0xDDDDDD), 0);
    lv_obj_set_style_text_opa(meter, LV_OPA_TRANSP, LV_PART_TICKS);

    /* Manually place labels 0–4 at the correct angular positions.
       The meter scale is lv_meter_set_scale_range(..., 0, 400, 240, 150): LVGL
       angles are clockwise from 3-o'clock (east), so value v sits at
       150 + (v/400)*240. For value = i*100 that's 150 + i*60 (i=0..4).
       Converting LVGL's clockwise-from-east angle to this loop's CCW-from-east
       trig convention (with ly = cy - r*sin) flips the sign: trig_angle =
       -(150 + i*60) mod 360 = 210 - i*60, which should put i=2 ("2") at exactly
       trig_angle=90 (straight up). In practice 210 still read slightly off on
       the physical display (207 was too far CW, 210 too far CCW), so this is
       tuned to 208.5 — re-adjust here if it still looks off. */
    static const char *tick_strs[] = { "0", "1", "2", "3", "4" };
    static const int label_r = 93;   /* radius from meter centre */
    static const int cx = 120;       /* meter centre x on screen */
    static const int cy = 120;       /* meter centre y on screen */
    for (int i = 0; i <= 4; i++) {
        double angle_deg = 208.5 - i * 60.0;
        double angle_rad = angle_deg * 3.14159265 / 180.0;
        int lx = (int)(cx + label_r * cos(angle_rad));
        int ly = (int)(cy - label_r * sin(angle_rad));
        lv_obj_t *lbl = lv_label_create(lv_scr_act());
        lv_label_set_text(lbl, tick_strs[i]);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, LV_PART_MAIN);
        lv_obj_set_pos(lbl, lx - 6, ly - 10);  /* offset to centre the glyph */
    }

    /* Red arc: 3500–4000 RPM — pushed outward */
    lv_meter_indicator_t *arc_red =
        lv_meter_add_arc(meter, scale, 10, lv_color_hex(0xCC0000), -4);
    lv_meter_set_indicator_start_value(meter, arc_red, 350);
    lv_meter_set_indicator_end_value(meter, arc_red, 400);

    /* Yellow arc: 3300–3500 RPM — same thickness, same outward position */
    lv_meter_indicator_t *arc_yellow =
        lv_meter_add_arc(meter, scale, 10, lv_color_hex(0xDDAA00), -4);
    lv_meter_set_indicator_start_value(meter, arc_yellow, 330);
    lv_meter_set_indicator_end_value(meter, arc_yellow, 350);

    /* White needle — position driven by rpm/10 in loop() */
    needle = lv_meter_add_needle_line(meter, scale, 3,
                                      lv_color_hex(0xEEEEEE), -10);
    lv_meter_set_indicator_start_value(meter, needle, 0);
    lv_meter_set_indicator_end_value(meter, needle, 0);

    /* "×1000 rpm" static label near top-centre */
    rpm_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(rpm_label, lv_color_hex(0xCCCCCC), LV_PART_MAIN);
    lv_label_set_text(rpm_label, "x1000 rpm");
    lv_obj_align(rpm_label, LV_ALIGN_CENTER, 0, -30);

    /* RPM readout box — dark rounded rectangle in the lower centre */
    lv_obj_t *rpm_box = lv_obj_create(lv_scr_act());
    lv_obj_set_size(rpm_box, 90, 30);
    lv_obj_align(rpm_box, LV_ALIGN_CENTER, 0, 60);
    lv_obj_set_style_bg_color(rpm_box, lv_color_hex(0x111111), LV_PART_MAIN);
    lv_obj_set_style_border_color(rpm_box, lv_color_hex(0x444444), LV_PART_MAIN);
    lv_obj_set_style_border_width(rpm_box, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(rpm_box, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_all(rpm_box, 0, LV_PART_MAIN);
    lv_obj_clear_flag(rpm_box, LV_OBJ_FLAG_SCROLLABLE);

    rpm_readout = lv_label_create(rpm_box);
    lv_label_set_text(rpm_readout, "0000");
    lv_obj_set_style_text_color(rpm_readout, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(rpm_readout, &lv_font_unscii_16, LV_PART_MAIN);
    lv_obj_center(rpm_readout);

    /* Direction arrows — below the RPM readout box, tips flush with its left/right
       edges. This keeps them in the gauge's dead zone (roughly the 120° arc
       between the "0" and "4" tick labels, where the scale has no ticks and the
       needle never sweeps) — the needle passes through the old pure-horizontal
       (62,0)/(-62,0) row at ~450 RPM (left) and ~3450 RPM (right), so it visibly
       crossed under a dark/lit arrow glyph there. */
    arrow_fwd = lv_label_create(lv_scr_act());
    lv_label_set_text(arrow_fwd, LV_SYMBOL_RIGHT LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(arrow_fwd, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
    lv_obj_set_style_text_font(arrow_fwd, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align_to(arrow_fwd, rpm_box, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 4);

    arrow_rev = lv_label_create(lv_scr_act());
    lv_label_set_text(arrow_rev, LV_SYMBOL_LEFT LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(arrow_rev, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
    lv_obj_set_style_text_font(arrow_rev, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align_to(arrow_rev, rpm_box, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);

    /* IP address overlay — dark pill centred on screen, hidden until button press */
    lv_obj_t *ip_box = lv_obj_create(lv_scr_act());
    lv_obj_set_size(ip_box, 210, 54);
    lv_obj_align(ip_box, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_bg_color(ip_box, lv_color_hex(0x111111), LV_PART_MAIN);
    lv_obj_set_style_border_color(ip_box, lv_color_hex(0x3b82d4), LV_PART_MAIN);
    lv_obj_set_style_border_width(ip_box, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(ip_box, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_all(ip_box, 6, LV_PART_MAIN);
    lv_obj_clear_flag(ip_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(ip_box, LV_OBJ_FLAG_HIDDEN);

    ip_overlay = lv_label_create(ip_box);
    lv_label_set_text(ip_overlay, "");
    lv_label_set_long_mode(ip_overlay, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(ip_overlay, 198);
    lv_obj_set_style_text_color(ip_overlay, lv_color_hex(0x3b82d4), LV_PART_MAIN);
    lv_obj_set_style_text_font(ip_overlay, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_align(ip_overlay, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_center(ip_overlay);
}

/* ---- Show overlay notification ---- */
static void show_overlay(const char *text, uint32_t duration_ms);   /* forward decl */

/* Update the direction arrow colours based on current VFD state.
   Running → bright green on whichever direction is active.
   Stopped → both arrows off — the motor isn't spinning, and the next
             Start could go either direction, so neither should be implied. */
static void update_direction_arrows()
{
    lv_color_t off_col = lv_color_hex(0x2A2A2A);

    /* Forward arrow */
    lv_color_t fwd_col = (vfd_running && !vfd_reverse_active)
                              ? lv_color_hex(0x00CC44)  /* bright green */
                              : off_col;
    lv_obj_set_style_text_color(arrow_fwd, fwd_col, LV_PART_MAIN);

    /* Reverse arrow */
    lv_color_t rev_col = (vfd_running && vfd_reverse_active)
                              ? lv_color_hex(0x00CC44)  /* bright green */
                              : off_col;
    lv_obj_set_style_text_color(arrow_rev, rev_col, LV_PART_MAIN);
}

/* ==========================================================================
   STEPPER DRIVER ABSTRACTION — TMC5160 implementation
   To substitute a different driver, replace stepper_init() and
   stepper_set_rpm() only. The rest of the firmware calls only these two.
   ========================================================================== */

/* TMC5160 VMAX conversion:
   VMAX [internal units] = RPM × MOTOR_STEPS × MICROSTEPS × 2^24 / 60 / fCLK
   fCLK = 12 MHz (internal oscillator) */
#define TMC_FCLK     12000000UL
#define RPM_TO_VMAX(rpm) \
    ((uint32_t)((float)(rpm) * MOTOR_STEPS * MICROSTEPS * 16777216.0f / 60.0f / TMC_FCLK))

/* Initialise the driver. Called once from setup() after the display is ready. */
static void stepper_init()
{
    pinMode(TMC_EN, OUTPUT);
    digitalWrite(TMC_EN, HIGH);     /* Keep disabled until fully configured */

    driver.begin();
    driver.reset();
    driver.rms_current(TMC_RMS_CURRENT);
    driver.microsteps(MICROSTEPS);
    driver.en_pwm_mode(true);       /* StealthChop */
    driver.pwm_autoscale(true);
    driver.RAMPMODE(1);             /* Start in positive velocity mode */
    driver.AMAX(500);               /* Acceleration limit (tune to taste) */
    driver.DMAX(500);               /* Deceleration limit */
    driver.VMAX(0);                 /* Start stopped */

    digitalWrite(TMC_EN, LOW);      /* Enable driver */
}

/* Set stepper shaft speed in RPM. Positive = forward, negative = reverse, 0 = stop. */
static void stepper_set_rpm(float rpm)
{
    uint32_t vmax = RPM_TO_VMAX(fabsf(rpm));
    if (rpm > 0.0f) {
        driver.RAMPMODE(1);
    } else if (rpm < 0.0f) {
        driver.RAMPMODE(2);
    } else {
        driver.VMAX(0);
        return;
    }
    driver.VMAX(vmax);
}

/* ==========================================================================
   END STEPPER DRIVER ABSTRACTION
   ========================================================================== */

/* ==========================================================================
   VFD DRIVER ABSTRACTION — Yaskawa A1000 Modbus RTU implementation, via
   SparkFun RS-485 Breakout on UART1.

   Unlike the stepper section above, this isn't a clean two-function swap:
   vfd_init(), vfd_run(), vfd_reverse(), vfd_stop(), vfd_reset_fault(),
   vfd_set_freq(), and vfd_poll() are ALL Yaskawa/Modbus-specific (register
   addresses, command/status bit layout — see a1000_modbus.h) and would all
   need reimplementing for a different VFD or a non-Modbus control scheme
   (e.g. 0-10V analog speed control). What the rest of the firmware — web
   handlers, physical buttons, the potentiometer, loop() — actually depends
   on is just that boundary: those seven functions, plus the state variables
   they maintain (vfd_running, vfd_reverse_active, vfd_status_word,
   vfd_fault_code, vfd_output_freq, vfd_comms_ok). Nothing outside this
   section should call vfd_send_frame()/vfd_recv_frame()/vfd_read_regs()/
   vfd_write_reg() or decode a status/command bit itself — if you find code
   outside this section doing that, it's a bug (this happened once already:
   handle_vfd_status() used to duplicate vfd_poll()'s register read instead
   of calling it).
   ========================================================================== */

/* Drive DE high, send frame, wait for all bytes to leave the UART FIFO,
   then drive DE low to release the bus for the response. */
static void vfd_send_frame(const uint8_t *frame, uint8_t len)
{
    digitalWrite(VFD_DE_PIN, HIGH);
    Serial2.write(frame, len);
    Serial2.flush();                /* blocks until TX shift register is empty */
    digitalWrite(VFD_DE_PIN, LOW);

    if (vfd_log_raw) {
        Serial.print("[VFD TX]");
        for (uint8_t i = 0; i < len; i++) {
            char tmp[4];
            snprintf(tmp, sizeof(tmp), " %02X", frame[i]);
            Serial.print(tmp);
        }
        Serial.println();
    }
}

/* Read up to max_len bytes with a timeout.  Returns actual byte count received.
   Timing budget: H5-06 transmit wait (default 5 ms, max 65 ms) + frame TX time.
   Worst case: 65 ms wait + 9 bytes × 11 bits/9600 baud ≈ 75 ms → 100 ms is safe. */
static uint8_t vfd_recv_frame(uint8_t *buf, uint8_t max_len, uint32_t timeout_ms = 100)
{
    uint32_t t   = millis();
    uint8_t  idx = 0;
    while (idx < max_len && (millis() - t) < timeout_ms) {
        if (Serial2.available()) {
            buf[idx++] = (uint8_t)Serial2.read();
            t = millis();           /* reset timeout on each received byte */
        }
    }

    if (vfd_log_raw) {
        if (idx > 0) {
            Serial.print("[VFD RX]");
            for (uint8_t i = 0; i < idx; i++) {
                char tmp[4];
                snprintf(tmp, sizeof(tmp), " %02X", buf[i]);
                Serial.print(tmp);
            }
            Serial.println();
        } else {
            Serial.println("[VFD RX] timeout — no bytes");
        }
    }
    return idx;
}

/* Flush any stale bytes left in the RX buffer before a new transaction */
static void vfd_flush_rx()
{
    while (Serial2.available()) Serial2.read();
}

/* FC 06 — Write Single Register.  Returns true on a valid echo response. */
static bool vfd_write_reg(uint16_t reg, uint16_t value)
{
    uint8_t frame[8];
    frame[0] = vfd_slave;
    frame[1] = 0x06;
    frame[2] = (uint8_t)(reg >> 8);
    frame[3] = (uint8_t)(reg & 0xFF);
    frame[4] = (uint8_t)(value >> 8);
    frame[5] = (uint8_t)(value & 0xFF);
    uint16_t crc = modbus_crc(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)(crc >> 8);

    vfd_flush_rx();
    vfd_send_frame(frame, 8);

    uint8_t resp[8];
    uint8_t n = vfd_recv_frame(resp, 8);
    if (n < 8) {
        Serial.printf("[VFD] FC06 reg=0x%04X val=0x%04X  FAIL (short response: %u bytes)\n",
                      reg, value, n);
        return false;
    }
    uint16_t resp_crc = (uint16_t)resp[7] << 8 | resp[6];
    if (modbus_crc(resp, 6) != resp_crc) {
        Serial.printf("[VFD] FC06 reg=0x%04X val=0x%04X  FAIL (bad CRC)\n", reg, value);
        return false;
    }
    bool ok = (resp[0] == frame[0] && resp[1] == 0x06 &&
               resp[2] == frame[2] && resp[3] == frame[3] &&
               resp[4] == frame[4] && resp[5] == frame[5]);
    Serial.printf("[VFD] FC06 reg=0x%04X val=0x%04X  %s\n",
                  reg, value, ok ? "OK" : "FAIL (echo mismatch)");
    return ok;
}

/* FC 03 — Read Holding Registers.  Fills out[] with `count` register values.
   Returns true on a valid response. */
static bool vfd_read_regs(uint16_t start_reg, uint8_t count, uint16_t *out)
{
    if (count == 0 || count > 16) return false;
    uint8_t frame[8];
    frame[0] = vfd_slave;
    frame[1] = 0x03;
    frame[2] = (uint8_t)(start_reg >> 8);
    frame[3] = (uint8_t)(start_reg & 0xFF);
    frame[4] = 0x00;
    frame[5] = count;
    uint16_t crc = modbus_crc(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)(crc >> 8);

    uint8_t resp_len = 5 + count * 2;   /* addr + FC + byte_count + data + 2×CRC */
    vfd_flush_rx();
    vfd_send_frame(frame, 8);

    uint8_t resp[40];
    uint8_t n = vfd_recv_frame(resp, resp_len);
    if (n < resp_len) {
        Serial.printf("[VFD] FC03 reg=0x%04X cnt=%u  FAIL (short response: %u bytes)\n",
                      start_reg, count, n);
        return false;
    }
    uint16_t resp_crc = (uint16_t)resp[n - 1] << 8 | resp[n - 2];
    if (modbus_crc(resp, n - 2) != resp_crc) {
        Serial.printf("[VFD] FC03 reg=0x%04X cnt=%u  FAIL (bad CRC)\n", start_reg, count);
        return false;
    }
    if (resp[0] != vfd_slave || resp[1] != 0x03) {
        Serial.printf("[VFD] FC03 reg=0x%04X cnt=%u  FAIL (unexpected slave/FC in response)\n",
                      start_reg, count);
        return false;
    }
    for (uint8_t i = 0; i < count; i++)
        out[i] = (uint16_t)resp[3 + i * 2] << 8 | resp[4 + i * 2];
    if (vfd_log_raw) {
        Serial.printf("[VFD] FC03 reg=0x%04X cnt=%u  OK", start_reg, count);
        for (uint8_t i = 0; i < count; i++) Serial.printf("  [%u]=0x%04X", i, out[i]);
        Serial.println();
    }
    return true;
}

/* ---- High-level VFD commands ---- */

/* Run forward at whatever frequency reference is currently set */
static bool vfd_run()
{
    Serial.println("[VFD] CMD run-forward");
    vfd_log_raw = true;
    bool ok = vfd_write_reg(A1000_REG_CMD_WORD, A1000_CMD_WORD_RUN_FORWARD);
    vfd_log_raw = false;
    if (ok) {
        vfd_running = true;
        vfd_reverse_active = false;
        vfd_dir_cmd_ms = millis();
    }
    return ok;
}

/* Run reverse */
static bool vfd_reverse()
{
    Serial.println("[VFD] CMD run-reverse");
    vfd_log_raw = true;
    bool ok = vfd_write_reg(A1000_REG_CMD_WORD, A1000_CMD_WORD_RUN_REVERSE);
    vfd_log_raw = false;
    if (ok) {
        vfd_running = true;
        vfd_reverse_active = true;
        vfd_dir_cmd_ms = millis();
    }
    return ok;
}

/* Stop (coast / decelerate per drive config) */
static bool vfd_stop()
{
    Serial.println("[VFD] CMD stop");
    vfd_log_raw = true;
    bool ok = vfd_write_reg(A1000_REG_CMD_WORD, A1000_CMD_WORD_STOP);
    vfd_log_raw = false;
    if (ok) vfd_running = false;
    return ok;
}

/* Reset active fault — rising edge on bit 3, then clear */
static bool vfd_reset_fault()
{
    Serial.println("[VFD] CMD fault-reset");
    vfd_log_raw = true;
    bool ok = vfd_write_reg(A1000_REG_CMD_WORD, A1000_CMD_FAULT_RESET);
    delay(50);
    ok &= vfd_write_reg(A1000_REG_CMD_WORD, A1000_CMD_WORD_STOP);
    vfd_log_raw = false;
    return ok;
}

/* Set frequency reference in 0.01 Hz units (e.g. 6000 = 60.00 Hz).
   Clamps to [0, vfd_max_hz]. */
static bool vfd_set_freq(uint16_t hz_hundredths)
{
    if (hz_hundredths > vfd_max_hz) hz_hundredths = vfd_max_hz;
    vfd_freq_ref = hz_hundredths;
    Serial.printf("[VFD] CMD set-freq %u (%.2f Hz)\n",
                  hz_hundredths, hz_hundredths / 100.0f);
    vfd_log_raw = true;
    bool ok = vfd_write_reg(A1000_REG_FREQ_REF, hz_hundredths);
    vfd_log_raw = false;
    return ok;
}

/* Decode fault code to a short string.  Returns pointer to a string literal. */
/* Called periodically from loop() — reads status word, fault code, and output frequency.
   Shows overlay on new faults or on fault clearance. */
static void vfd_poll()
{
    /* Read Drive Status 1 (A1000_REG_STATUS_1) through Output Frequency
       (A1000_REG_OUTPUT_FREQ) in one transaction — these five registers are
       contiguous (0x0020-0x0024). A1000_REG_FAULT_BITMASK_1 falls inside
       this window but isn't a decodable fault code (see a1000_modbus.h), so
       it's read but unused here. */
    uint16_t regs[5];
    bool ok = vfd_read_regs(A1000_REG_STATUS_1, 5, regs);
    bool was_ok = vfd_comms_ok;
    vfd_comms_ok = ok;
    if (!ok) {
        if (was_ok) Serial.println("[VFD] poll — comms lost");
        return;
    }

    uint16_t new_status      = regs[0];   /* A1000_REG_STATUS_1     (0x0020) */
    uint16_t new_output_freq = regs[4];   /* A1000_REG_OUTPUT_FREQ  (0x0024) */

    /* The drive's actual decodable fault code lives at a separate,
       non-contiguous register (A1000_REG_CURRENT_FAULT). Only fetch it when
       the status word's Fault bit is set, so a healthy drive costs one
       Modbus transaction per poll instead of two. Default to the last known
       value (not 0) so a transient failure on just this second read doesn't
       misreport an active fault as cleared. */
    uint16_t new_fault = vfd_fault_code;
    if ((new_status & A1000_STATUS1_FAULT) != 0) {
        uint16_t fault_reg[1];
        if (vfd_read_regs(A1000_REG_CURRENT_FAULT, 1, fault_reg)) new_fault = fault_reg[0];
        /* else: leave new_fault as the last known value — don't claim cleared */
    } else {
        new_fault = 0;   /* status word confirms no active fault */
    }

    /* Log any change in status word or fault code */
    if (new_status != vfd_status_word || new_fault != vfd_fault_code) {
        Serial.printf("[VFD] poll  status=0x%04X  fault=0x%04X (%s)  running=%d\n",
                      new_status, new_fault,
                      new_fault ? a1000_current_fault_name(new_fault) : "none",
                      (new_status & A1000_STATUS1_DURING_RUN) != 0);
    }

    /* Fault appeared */
    if (new_fault != 0 && new_fault != vfd_fault_code) {
        char msg[48];
        snprintf(msg, sizeof(msg), "VFD Fault: %s", a1000_current_fault_name(new_fault));
        show_overlay(msg, 6000);
    }
    /* Fault cleared */
    if (new_fault == 0 && vfd_fault_code != 0) {
        show_overlay("VFD Fault Cleared", 3000);
    }

    vfd_status_word = new_status;
    vfd_fault_code  = new_fault;
    vfd_output_freq = new_output_freq;
    vfd_running     = (new_status & A1000_STATUS1_DURING_RUN) != 0;
    /* Sync intended direction from drive only while running — when stopped the
       drive reports "During Reverse" = 0 regardless, so we preserve
       vfd_reverse_active so Start correctly resumes in the last-running
       direction. Skip the sync for a grace period after we issue a direction
       command: the drive ramps to 0 before actually reversing, so a poll
       mid-ramp would still report the old direction and clobber the command
       we just sent, causing the arrow to flash back to the old direction
       before settling correctly. */
    if (vfd_running && (millis() - vfd_dir_cmd_ms >= VFD_DIR_SYNC_GRACE_MS))
        vfd_reverse_active = (new_status & A1000_STATUS1_DURING_REVERSE) != 0;
}

/* Initialise UART and the DE pin.  Called once from setup().
   Serial2 maps to HW UART1 on GPIO 8/9 in the rpipico2w variant
   (PIN_SERIAL2_TX=8, PIN_SERIAL2_RX=9). */
static void vfd_init()
{
    pinMode(VFD_DE_PIN, OUTPUT);
    digitalWrite(VFD_DE_PIN, LOW);   /* receive mode */
    Serial2.begin(VFD_BAUD, SERIAL_8N2);
    Serial.println("[VFD] init — UART1 9600 8-N-2 ready");
}

/* ==========================================================================
   END VFD DRIVER ABSTRACTION
   ========================================================================== */

/* ---- Compute required stepper RPM from encoder RPM and apply it ---- */
static void set_stepper_rpm(int32_t signed_rpm)
{
    /* stepper must produce the hobbing-ratio output at the final shaft, corrected for belt/pulley */
    float hobbing     = (float)hob_threads   / (float)gear_teeth;
    float belt        = (float)pulley_driven / (float)pulley_driver;
    float target_rpm  = (float)signed_rpm * hobbing * belt;
    stepper_set_rpm(target_rpm);
}

/* ---- Startup sweep animation ---- */
static void needle_anim_cb(void *obj, int32_t val)
{
    /* val is the scale value (0–400); update needle and readout in sync */
    lv_meter_set_indicator_end_value(meter, needle, val);

    char buf[8];
    snprintf(buf, sizeof(buf), "%04lu", (uint32_t)(val * 10));
    lv_label_set_text(rpm_readout, buf);
}

static void run_startup_animation(void)
{
    /* Sweep up 0 → 400 in 1200 ms, then back 400 → 0 in 1200 ms */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_exec_cb(&a, needle_anim_cb);
    lv_anim_set_var(&a, NULL);          /* unused — we access globals directly */
    lv_anim_set_values(&a, 0, 400);
    lv_anim_set_time(&a, 1200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_set_playback_time(&a, 1200);
    lv_anim_set_playback_delay(&a, 100);
    lv_anim_start(&a);

    /* Run LVGL until the full animation (sweep up + sweep down) completes.
       Total duration = time + playback_delay + playback_time = 2500 ms */
    uint32_t start = millis();
    uint32_t last_tick = start;
    while (millis() - start < 2600)
    {
        uint32_t now = millis();
        lv_tick_inc(now - last_tick);
        last_tick = now;
        lv_timer_handler();
        delay(5);
    }

    /* Reset needle and readout to zero before handing off to loop() */
    lv_meter_set_indicator_end_value(meter, needle, 0);
    lv_label_set_text(rpm_readout, "0000");
}

/* ---- Web UI ---- */
static const String WEB_PAGE =
    "<!DOCTYPE html><html lang='en'><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Machine Tool Gizmo</title>"
    "<style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{font-family:-apple-system,sans-serif;background:#111;color:#eee;"
    "display:flex;flex-direction:column;align-items:center;"
    "min-height:100vh;padding:0}"
    /* tab bar */
    ".tabs{display:flex;width:100%;max-width:420px;border-bottom:1px solid #333;"
    "margin-top:24px}"
    ".tab{flex:1;padding:12px 0;text-align:center;font-size:.9rem;font-weight:600;"
    "color:#666;cursor:pointer;border-bottom:2px solid transparent;transition:.15s}"
    ".tab.active{color:#fff;border-bottom-color:#3b82d4}"
    /* pages */
    ".page{display:none;flex-direction:column;align-items:center;"
    "width:100%;max-width:420px;gap:20px;padding:24px 16px 40px}"
    ".page.active{display:flex}"
    /* cards */
    "h1{font-size:1.3rem;font-weight:600;color:#fff;align-self:flex-start;padding:0 4px}"
    ".card{background:#1e1e1e;border:1px solid #333;border-radius:10px;"
    "padding:24px 28px;width:100%;display:flex;flex-direction:column;gap:16px}"
    "h2{font-size:.9rem;font-weight:600;color:#aaa}"
    "label{font-size:.78rem;color:#888;text-transform:uppercase;letter-spacing:.08em}"
    ".current{font-size:2.4rem;font-weight:700;color:#3b82d4;text-align:center;"
    "font-variant-numeric:tabular-nums}"
    "input[type=number],input[type=text],input[type=password]{"
    "width:100%;padding:10px 14px;background:#2a2a2a;border:1px solid #444;"
    "border-radius:6px;color:#fff;font-size:1rem;outline:none}"
    "input:focus{border-color:#3b82d4}"
    "select{width:100%;padding:10px 14px;background:#2a2a2a;border:1px solid #444;"
    "border-radius:6px;color:#fff;font-size:1rem;outline:none}"
    ".presets{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}"
    ".preset{padding:8px;background:#2a2a2a;border:1px solid #444;border-radius:6px;"
    "color:#ccc;font-size:.8rem;cursor:pointer;text-align:center}"
    ".preset:hover{border-color:#3b82d4;color:#fff}"
    ".toggle{display:flex;align-items:center;gap:12px;cursor:pointer}"
    ".toggle input{display:none}"
    ".slider{width:44px;height:24px;background:#444;border-radius:12px;"
    "position:relative;transition:.2s}"
    ".slider::after{content:'';position:absolute;width:18px;height:18px;"
    "background:#fff;border-radius:50%;top:3px;left:3px;transition:.2s}"
    "input:checked+.slider{background:#3b82d4}"
    "input:checked+.slider::after{left:23px}"
    ".tlabel{font-size:.9rem;color:#ccc}"
    "button[type=submit]{width:100%;padding:12px;background:#3b82d4;border:none;"
    "border-radius:6px;color:#fff;font-size:1rem;font-weight:600;cursor:pointer}"
    "button[type=submit]:hover{background:#2563eb}"
    ".msg{font-size:.85rem;text-align:center;color:#4ade80;min-height:1.2em}"
    ".err{color:#f87171}"
    ".info{font-size:.75rem;color:#666;text-align:center}"
    "</style></head><body>"
    /* ---- Tab bar ---- */
    "<div class='tabs'>"
    "<div class='tab active' id='tabHome' onclick='showPage(\"home\")'>Home</div>"
    "<div class='tab' id='tabSettings' onclick='showPage(\"settings\")'>Settings</div>"
    "</div>"
    /* ---- Home page ---- */
    "<div class='page active' id='pageHome'>"
    "<h1>Machine Tool Gizmo</h1>"
    /* ---- VFD card (Home tab) ---- */
    "<div class='card'>"
    "<h2>VFD — Yaskawa A1000</h2>"
    "<label>Status</label>"
    "<div id='vfdStatus' style='font-size:1.1rem;font-weight:600;color:#aaa'>--</div>"
    "<label style='margin-top:4px'>VFD Speed</label>"
    "<div id='vfdRpm' style='font-size:1.1rem;color:#aaa'>-- RPM</div>"
    "<label style='margin-top:4px'>Set Speed</label>"
    "<div id='vfdSetpoint' style='font-size:1.1rem;color:#aaa'>-- RPM</div>"
    "<label style='margin-top:4px'>Speed Setting (RPM)</label>"
    "<div id='vfdRpmRow' style='display:flex;gap:8px;margin-top:6px'>"
    "<input type='number' id='vfdRpmIn' min='0' step='1'"
    " placeholder='e.g. 1750' style='flex:1'>"
    "<button type='button' onclick='vfdSetRpm()' id='vfdRpmBtn'"
    " style='padding:0 14px;background:#2a2a2a;border:1px solid #444;"
    "border-radius:6px;color:#ccc;cursor:pointer;font-size:.9rem;"
    "white-space:nowrap'>Set RPM</button>"
    "</div>"
    "<div id='vfdCtrlBtns' style='display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px'>"
    "<button type='button' onclick='vfdCmd(\"/vfd-run\")'"
    " style='padding:10px;background:#166534;border:none;"
    "border-radius:6px;color:#fff;font-size:.9rem;font-weight:600;"
    "cursor:pointer'>\u25b6 Forward</button>"
    "<button type='button' onclick='vfdCmd(\"/vfd-reverse\")'"
    " style='padding:10px;background:#166534;border:none;"
    "border-radius:6px;color:#fff;font-size:.9rem;font-weight:600;"
    "cursor:pointer'>\u25c0 Reverse</button>"
    "<button type='button' onclick='vfdCmd(\"/vfd-stop\")'"
    " style='padding:10px;background:#7f1d1d;border:none;"
    "border-radius:6px;color:#fff;font-size:.9rem;font-weight:600;"
    "cursor:pointer'>Stop \u25a0</button>"
    "<button type='button' onclick='vfdCmd(\"/vfd-reset\")'"
    " style='padding:10px;background:#2a2a2a;border:1px solid #444;"
    "border-radius:6px;color:#ccc;font-size:.9rem;cursor:pointer'>Reset</button>"
    "</div>"
    "<div id='vfdLockNote' style='display:none;font-size:.8rem;color:#888;"
    "margin-top:4px'>Controls locked — unlock jumper not present</div>"
    "<div class='msg' id='msgVfd'></div>"
    "</div>"
    "<div class='card'>"
    "<h2>Gear Ratio</h2>"
    "<label>Current ratio</label>"
    "<div class='current' id='cur'>__HOB__ : __TEETH__</div>"
    "<form id='fRatio'>"
    "<label for='hobVal'>Threads on hob</label>"
    "<input type='number' id='hobVal' name='hob' min='1' max='9999' step='1'"
    " placeholder='e.g. 1' required style='margin-top:6px'>"
    "<label for='teethVal' style='margin-top:10px'>Gear teeth</label>"
    "<input type='number' id='teethVal' name='teeth' min='1' max='9999' step='1'"
    " placeholder='e.g. 32' required style='margin-top:6px'>"
    "<button type='submit' style='margin-top:14px'>Set Ratio</button>"
    "</form>"
    "<div class='msg' id='msgRatio'></div>"
    "</div>"
    "</div>"
    /* ---- Settings page ---- */
    "<div class='page' id='pageSettings'>"
    "<h1>Settings</h1>"
    /* Encoder card */
    "<div class='card'>"
    "<h2>Encoder</h2>"
    "<label class='toggle'>"
    "<input type='checkbox' id='chkRev' __REVCHECKED__>"
    "<span class='slider'></span>"
    "<span class='tlabel'>Reverse direction</span>"
    "</label>"
    "<div class='msg' id='msgEnc'></div>"
    "</div>"
    /* PPR card */
    "<div class='card'>"
    "<h2>Encoder PPR</h2>"
    "<label>Current pulses per revolution</label>"
    "<div class='current' id='curPpr' style='font-size:2rem'>__PPR__</div>"
    "<form id='fPpr'>"
    "<label for='pprVal'>New PPR</label>"
    "<input type='number' id='pprVal' name='ppr' min='1' max='10000' step='1'"
    " placeholder='e.g. 600' required style='margin-top:6px'>"
    "<div class='presets' style='margin-top:10px'>"
    "<div class='preset' onclick='setPpr(100)'>100</div>"
    "<div class='preset' onclick='setPpr(200)'>200</div>"
    "<div class='preset' onclick='setPpr(400)'>400</div>"
    "<div class='preset' onclick='setPpr(600)'>600</div>"
    "<div class='preset' onclick='setPpr(1000)'>1000</div>"
    "<div class='preset' onclick='setPpr(2400)'>2400</div>"
    "</div>"
    "<button type='submit' style='margin-top:14px'>Set PPR</button>"
    "</form>"
    "<div class='msg' id='msgPpr'></div>"
    "</div>"
    /* Pulley / belt card */
    "<div class='card'>"
    "<h2>Stepper Pulley</h2>"
    "<label>Driver (stepper) : Driven (output)</label>"
    "<div class='current' id='curPulley'>__PULLEY_DRV__ : __PULLEY_DRN__</div>"
    "<form id='fPulley'>"
    "<label for='drvVal'>Driver teeth (stepper pulley)</label>"
    "<input type='number' id='drvVal' name='driver' min='1' max='9999' step='1'"
    " placeholder='e.g. 20' required style='margin-top:6px'>"
    "<label for='drnVal' style='margin-top:10px'>Driven teeth (output pulley)</label>"
    "<input type='number' id='drnVal' name='driven' min='1' max='9999' step='1'"
    " placeholder='e.g. 20' required style='margin-top:6px'>"
    "<button type='submit' style='margin-top:14px'>Set Pulley</button>"
    "</form>"
    "<div class='msg' id='msgPulley'></div>"
    "</div>"
    /* WiFi card */
    "<div class='card'>"
    "<h2>WiFi</h2>"
    "<div class='info'>Current IP: __IP__ (__WIFIMODE__)</div>"
    "<form id='fWifi'>"
    "<label for='wmode'>Mode</label>"
    "<select id='wmode' name='mode' onchange='toggleSTA()' style='margin-top:6px'>"
    "<option value='ap' __APSEL__>Access Point (AP)</option>"
    "<option value='sta' __STASEL__>Station (connect to router)</option>"
    "</select>"
    "<div id='staFields' style='margin-top:10px;flex-direction:column;gap:10px'>"
    "<div>"
    "<div style='display:flex;justify-content:space-between;align-items:center'>"
    "<label>SSID</label>"
    "<button type='button' onclick='scanWifi()' id='btnScan'"
    " style='padding:2px 10px;background:#2a2a2a;border:1px solid #444;border-radius:6px;"
    "color:#ccc;cursor:pointer;font-size:.8rem;white-space:nowrap'>Scan</button>"
    "</div>"
    "<select id='ssidSelect' style='margin-top:4px;width:100%' onchange='document.getElementById(\"wssid\").value=this.value'>"
    "<option value=''>-- select network --</option>"
    "</select>"
    "<input type='text' id='wssid' name='ssid'"
    " placeholder='Or type network name' style='margin-top:6px'>"
    "</div>"
    "<div><label for='wpass'>Password</label>"
    "<div style='display:flex;gap:8px;margin-top:4px'>"
    "<input type='password' id='wpass' name='pass' placeholder='Password' style='flex:1'>"
    "<button type='button' id='btnShow' onclick='togglePw()'"
    " style='padding:0 12px;background:#2a2a2a;border:1px solid #444;border-radius:6px;"
    "color:#ccc;cursor:pointer;font-size:.8rem;white-space:nowrap'>Show</button>"
    "</div></div>"
    "</div>"
    "<button type='submit' style='margin-top:14px'>Apply WiFi</button>"
    "</form>"
    "<div class='msg' id='msgWifi'></div>"
    "</div>"
    /* VFD Settings card */
    "<div class='card'>"
    "<h2>VFD Settings</h2>"
    "<form id='fVfdSettings'>"
    "<label for='vfdSlave'>Modbus slave address (1–31)</label>"
    "<input type='number' id='vfdSlave' name='slave' min='1' max='31' step='1'"
    " value='__VFDSLAVE__' required style='margin-top:6px'>"
    "<label for='vfdMaxHz' style='margin-top:10px'>Max frequency (Hz, 0.01–400.00)</label>"
    "<input type='number' id='vfdMaxHz' name='maxhz' min='0.01' max='400' step='0.01'"
    " value='__VFDMAXHZ__' required style='margin-top:6px'>"
    "<label for='vfdBaseHz' style='margin-top:10px'>Baseline frequency (Hz)</label>"
    "<input type='number' id='vfdBaseHz' name='basehz' min='0.01' max='400' step='0.01'"
    " value='__VFDBASEHZ__' required style='margin-top:6px'>"
    "<label for='vfdBaseRpm' style='margin-top:10px'>RPM at baseline frequency</label>"
    "<input type='number' id='vfdBaseRpm' name='baserpm' min='1' max='60000' step='1'"
    " value='__VFDBASERPM__' required style='margin-top:6px'>"
    "<button type='submit' style='margin-top:14px'>Save VFD Settings</button>"
    "</form>"
    "<div class='msg' id='msgVfdSettings'></div>"
    "</div>"
    "</div>"
    "<script>"
    /* page switching */
    "function showPage(p){"
    "document.getElementById('pageHome').classList.toggle('active',p==='home');"
    "document.getElementById('pageSettings').classList.toggle('active',p==='settings');"
    "document.getElementById('tabHome').classList.toggle('active',p==='home');"
    "document.getElementById('tabSettings').classList.toggle('active',p==='settings');"
    "}"
    /* ratio form */
    "document.getElementById('fRatio').addEventListener('submit',async function(e){"
    "e.preventDefault();"
    "var h=parseInt(document.getElementById('hobVal').value);"
    "var t=parseInt(document.getElementById('teethVal').value);"
    "if(isNaN(h)||h<1||isNaN(t)||t<1){showMsg('msgRatio','Invalid value',true);return;}"
    "var res=await fetch('/set?hob='+h+'&teeth='+t,{method:'POST'});"
    "var txt=await res.text();"
    "document.getElementById('cur').textContent=h+' : '+t;"
    "showMsg('msgRatio',txt,false);"
    "});"
    /* encoder toggle */
    "document.getElementById('chkRev').addEventListener('change',async function(){"
    "var v=this.checked?'1':'0';"
    "var res=await fetch('/set-encoder?reversed='+v,{method:'POST'});"
    "var txt=await res.text();"
    "showMsg('msgEnc',txt,false);"
    "});"
    /* PPR form */
    "function setPpr(v){document.getElementById('pprVal').value=v;}"
    "document.getElementById('fPpr').addEventListener('submit',async function(e){"
    "e.preventDefault();"
    "var v=parseInt(document.getElementById('pprVal').value);"
    "if(isNaN(v)||v<1||v>10000){showMsg('msgPpr','Must be 1–10000',true);return;}"
    "var res=await fetch('/set-ppr?ppr='+v,{method:'POST'});"
    "var txt=await res.text();"
    "document.getElementById('curPpr').textContent=v;"
    "showMsg('msgPpr',txt,false);"
    "});"
    /* wifi mode toggle */
    "function toggleSTA(){"
    "var m=document.getElementById('wmode').value;"
    "document.getElementById('staFields').style.display=(m==='sta'?'flex':'none');"
    "if(m==='sta')scanWifi();"
    "}"
    "toggleSTA();"
    "async function scanWifi(){"
    "var btn=document.getElementById('btnScan');"
    "btn.textContent='Scanning...';btn.disabled=true;"
    "var sel=document.getElementById('ssidSelect');"
    "sel.innerHTML='<option value=\"\">-- scanning --</option>';"
    "try{"
    "var r=await fetch('/wifi-scan');"
    "var d=await r.json();"
    "sel.innerHTML='<option value=\"\">-- select network --</option>';"
    "if(d.length===0){"
    "sel.innerHTML='<option value=\"\">No networks found</option>';}"
    "else{"
    "d.forEach(function(s){"
    "var o=document.createElement('option');o.value=s;o.textContent=s;sel.appendChild(o);"
    "});}"
    "}catch(e){"
    "sel.innerHTML='<option value=\"\">Scan failed</option>';}"
    "btn.textContent='Scan';btn.disabled=false;"
    "}"
    /* password reveal */
    "function togglePw(){"
    "var inp=document.getElementById('wpass');"
    "var btn=document.getElementById('btnShow');"
    "if(inp.type==='password'){inp.type='text';btn.textContent='Hide';}"
    "else{inp.type='password';btn.textContent='Show';}"
    "}"
    /* wifi form */
    "document.getElementById('fWifi').addEventListener('submit',async function(e){"
    "e.preventDefault();"
    "var m=document.getElementById('wmode').value;"
    "var s=document.getElementById('wssid').value;"
    "var p=document.getElementById('wpass').value;"
    "if(m==='sta'&&!s){showMsg('msgWifi','SSID required',true);return;}"
    "var url='/set-wifi?mode='+encodeURIComponent(m)"
    "+'&ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p);"
    "showMsg('msgWifi','Applying... reconnect if needed.',false);"
    "try{await fetch(url,{method:'POST'});}catch(e){}"
    "});"
    /* pulley form */
    "document.getElementById('fPulley').addEventListener('submit',async function(e){"
    "e.preventDefault();"
    "var d=parseInt(document.getElementById('drvVal').value);"
    "var n=parseInt(document.getElementById('drnVal').value);"
    "if(isNaN(d)||d<1||isNaN(n)||n<1){showMsg('msgPulley','Invalid value',true);return;}"
    "var res=await fetch('/set-pulley?driver='+d+'&driven='+n,{method:'POST'});"
    "var txt=await res.text();"
    "document.getElementById('curPulley').textContent=d+' : '+n;"
    "showMsg('msgPulley',txt,false);"
    "});"
    /* helper */
    "function showMsg(id,txt,err){"
    "var el=document.getElementById(id);"
    "el.textContent=txt;el.className=err?'msg err':'msg';"
    "setTimeout(function(){el.textContent='';},4000);"
    "}"
    /* VFD status poll (every 500 ms while on Home tab) */
    "var vfdPollTimer=null;"
    "function startVfdPoll(){"
    "if(vfdPollTimer)return;"
    "vfdPollTimer=setInterval(async function(){"
    "try{"
    "var r=await fetch('/vfd-status');"
    "var d=await r.json();"
    "var statusTxt='No comms';"
    "if(d.comms_ok){"
    "if(d.running)statusTxt=d.reverse?'\u25c0 Running Reverse':'Running Forward \u25b6';"
    "else statusTxt='Stopped';"
    "}"
    "document.getElementById('vfdStatus').textContent=statusTxt;"
    "document.getElementById('vfdStatus').style.color="
    "d.comms_ok?(d.running?'#4ade80':'#aaa'):'#f87171';"
    "var outRpm=d.output_freq>0?Math.round(d.output_freq*d.base_rpm/d.base_hz):0;"
    "document.getElementById('vfdRpm').textContent=(d.comms_ok?outRpm+' RPM':'-- RPM');"
    "var setRpm=d.freq>0?Math.round(d.freq*d.base_rpm/d.base_hz):0;"
    "document.getElementById('vfdSetpoint').textContent=(d.comms_ok?setRpm+' RPM':'-- RPM');"
    "var locked=!!d.web_lock;"
    "document.getElementById('vfdRpmIn').disabled=locked;"
    "document.getElementById('vfdRpmBtn').disabled=locked;"
    "document.getElementById('vfdRpmRow').style.opacity=locked?'0.35':'1';"
    "var btns=document.getElementById('vfdCtrlBtns').querySelectorAll('button');"
    "btns.forEach(function(b){b.disabled=locked;});"
    "document.getElementById('vfdCtrlBtns').style.opacity=locked?'0.35':'1';"
    "document.getElementById('vfdLockNote').style.display=locked?'block':'none';"
    "}catch(e){}"
    "},500);"
    "}"
    "function stopVfdPoll(){clearInterval(vfdPollTimer);vfdPollTimer=null;}"
    /* patch showPage to start/stop VFD poll */
    "var _origShowPage=showPage;"
    "showPage=function(p){"
    "_origShowPage(p);"
    "if(p==='home')startVfdPoll();else stopVfdPoll();"
    "};"
    "startVfdPoll();"
    /* Simple fire-and-forget VFD command */
    "async function vfdCmd(url){"
    "try{"
    "var r=await fetch(url,{method:'POST'});"
    "var t=await r.text();"
    "showMsg('msgVfd',t,!r.ok);"
    "}catch(e){showMsg('msgVfd','Error',true);}"
    "}"
    /* VFD set speed in RPM — firmware converts to Hz */
    "async function vfdSetRpm(){"
    "var v=parseInt(document.getElementById('vfdRpmIn').value);"
    "if(isNaN(v)||v<0){showMsg('msgVfd','Enter a valid RPM',true);return;}"
    "try{"
    "var r=await fetch('/vfd-rpm?rpm='+v,{method:'POST'});"
    "var t=await r.text();"
    "showMsg('msgVfd',t,!r.ok);"
    "}catch(e){showMsg('msgVfd','Error',true);}"
    "}"
    /* VFD settings form */
    "document.getElementById('fVfdSettings').addEventListener('submit',async function(e){"
    "e.preventDefault();"
    "var s=parseInt(document.getElementById('vfdSlave').value);"
    "var m=parseFloat(document.getElementById('vfdMaxHz').value);"
    "var bh=parseFloat(document.getElementById('vfdBaseHz').value);"
    "var br=parseInt(document.getElementById('vfdBaseRpm').value);"
    "if(isNaN(s)||s<1||s>31){showMsg('msgVfdSettings','Slave must be 1–31',true);return;}"
    "if(isNaN(m)||m<=0||m>400){showMsg('msgVfdSettings','Max Hz must be 0.01–400',true);return;}"
    "if(isNaN(bh)||bh<=0||bh>400){showMsg('msgVfdSettings','Baseline Hz must be 0.01–400',true);return;}"
    "if(isNaN(br)||br<1||br>60000){showMsg('msgVfdSettings','Baseline RPM must be 1–60000',true);return;}"
    "var cents=Math.round(m*100);"
    "var bcents=Math.round(bh*100);"
    "var r=await fetch('/vfd-settings?slave='+s+'&maxhz='+cents+'&basehz='+bcents+'&baserpm='+br,{method:'POST'});"
    "var t=await r.text();"
    "showMsg('msgVfdSettings',t,!r.ok);"
    "});"
    "</script></body></html>";

static void apply_wifi_config();   /* forward declarations */
static void save_config();

static void handle_root()
{
    String page = WEB_PAGE;
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)hob_threads);
    page.replace("__HOB__", buf);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)gear_teeth);
    page.replace("__TEETH__", buf);
    page.replace("__REVCHECKED__", encoder_reversed ? "checked" : "");
    char ppr_buf[8];
    snprintf(ppr_buf, sizeof(ppr_buf), "%lu", (unsigned long)encoder_ppr);
    page.replace("__PPR__", ppr_buf);
    char pulley_drv_buf[8], pulley_drn_buf[8];
    snprintf(pulley_drv_buf, sizeof(pulley_drv_buf), "%lu", (unsigned long)pulley_driver);
    snprintf(pulley_drn_buf, sizeof(pulley_drn_buf), "%lu", (unsigned long)pulley_driven);
    page.replace("__PULLEY_DRV__", pulley_drv_buf);
    page.replace("__PULLEY_DRN__", pulley_drn_buf);
    page.replace("__IP__",       current_ip);
    page.replace("__WIFIMODE__", current_wifi_mode == MODE_AP ? "AP" : "STA");
    page.replace("__APSEL__",    wifi_mode == MODE_AP  ? "selected" : "");
    page.replace("__STASEL__",   wifi_mode == MODE_STA ? "selected" : "");
    /* VFD settings placeholders */
    char vfd_slave_buf[4];
    snprintf(vfd_slave_buf, sizeof(vfd_slave_buf), "%u", (unsigned)vfd_slave);
    page.replace("__VFDSLAVE__", vfd_slave_buf);
    char vfd_maxhz_buf[10];
    snprintf(vfd_maxhz_buf, sizeof(vfd_maxhz_buf), "%.2f", vfd_max_hz / 100.0f);
    page.replace("__VFDMAXHZ__", vfd_maxhz_buf);
    char vfd_basehz_buf[10];
    snprintf(vfd_basehz_buf, sizeof(vfd_basehz_buf), "%.2f", vfd_base_hz / 100.0f);
    page.replace("__VFDBASEHZ__", vfd_basehz_buf);
    char vfd_baserpm_buf[8];
    snprintf(vfd_baserpm_buf, sizeof(vfd_baserpm_buf), "%u", (unsigned)vfd_base_rpm);
    page.replace("__VFDBASERPM__", vfd_baserpm_buf);
    server.send(200, "text/html", page);
}

/* CSRF guard: browsers always send Origin on cross-site POSTs (it can't be
   spoofed by page JS), so reject any request whose Origin doesn't match this
   device. A request with no Origin header at all (curl, direct API clients)
   is allowed through — there's no session/token scheme here to check instead,
   so this only stops the classic "malicious page silently POSTs to a known
   LAN IP" attack, not a deliberate direct request. */
static bool origin_ok()
{
    String origin = server.header("Origin");
    if (origin.length() == 0) return true;
    return origin == ("http://" + current_ip);
}

/* Physical lockout jumper (VFD_LOCK_BTN, INPUT_PULLUP). Fail-safe: the pin
   must be actively pulled LOW by the jumper being physically present to
   UNLOCK. Any absence of that connection — jumper not installed, wire
   broken, connector unplugged — reads HIGH and defaults to LOCKED. This is
   the opposite sense of a normal "hold to disable" switch on purpose: a
   safety interlock must fail toward the safe state, not the enabled one.
   Must be checked inside every handler that can start/reverse/reconfigure
   the spindle — the web UI only grays out buttons client-side, which is not
   a real guarantee. Stop is deliberately never gated by this: a lockout must
   never be able to block the one command that makes the machine safer. */
static bool vfd_web_locked()
{
    return digitalRead(VFD_LOCK_BTN) == HIGH;
}

/* POST /set — sets the hobbing ratio (hob threads : gear teeth) */
static void handle_set()
{
    if (!origin_ok()) { server.send(403, "text/plain", "Forbidden"); return; }
    if (!server.hasArg("hob") || !server.hasArg("teeth")) {
        server.send(400, "text/plain", "Missing hob or teeth parameter");
        return;
    }
    uint32_t hob = (uint32_t)server.arg("hob").toInt();
    uint32_t tth = (uint32_t)server.arg("teeth").toInt();
    if (hob < 1 || hob > 9999 || tth < 1 || tth > 9999) {
        server.send(400, "text/plain", "hob and teeth must be 1–9999");
        return;
    }
    hob_threads = hob;
    gear_teeth  = tth;
    char buf[48];
    snprintf(buf, sizeof(buf), "Ratio set to %lu:%lu", (unsigned long)hob, (unsigned long)tth);
    server.send(200, "text/plain", buf);
    save_config();
}

static void handle_set_encoder()
{
    if (!origin_ok()) { server.send(403, "text/plain", "Forbidden"); return; }
    if (!server.hasArg("reversed")) {
        server.send(400, "text/plain", "Missing parameter");
        return;
    }
    encoder_reversed = (server.arg("reversed") == "1");
    server.send(200, "text/plain",
                encoder_reversed ? "Encoder reversed" : "Encoder normal");
    save_config();
}

static void handle_set_ppr()
{
    if (!origin_ok()) { server.send(403, "text/plain", "Forbidden"); return; }
    if (!server.hasArg("ppr")) {
        server.send(400, "text/plain", "Missing ppr parameter");
        return;
    }
    int val = server.arg("ppr").toInt();
    if (val < 1 || val > 10000) {
        server.send(400, "text/plain", "PPR must be between 1 and 10000");
        return;
    }
    encoder_ppr = (uint32_t)val;
    char buf[32];
    snprintf(buf, sizeof(buf), "PPR set to %d", val);
    server.send(200, "text/plain", buf);
    save_config();
}

static void handle_set_pulley()
{
    if (!origin_ok()) { server.send(403, "text/plain", "Forbidden"); return; }
    if (!server.hasArg("driver") || !server.hasArg("driven")) {
        server.send(400, "text/plain", "Missing driver or driven parameter");
        return;
    }
    uint32_t drv = (uint32_t)server.arg("driver").toInt();
    uint32_t drn = (uint32_t)server.arg("driven").toInt();
    if (drv < 1 || drv > 9999 || drn < 1 || drn > 9999) {
        server.send(400, "text/plain", "Pulley teeth must be 1–9999");
        return;
    }
    pulley_driver = drv;
    pulley_driven = drn;
    char buf[48];
    snprintf(buf, sizeof(buf), "Pulley set to %lu:%lu", (unsigned long)drv, (unsigned long)drn);
    server.send(200, "text/plain", buf);
    save_config();
}

/* ip_show_until is defined in loop() as a static — declare it here so show_overlay can set it */
static uint32_t g_ip_show_until = 0;

/* ---- VFD web handlers ---- */

static void handle_vfd_run()
{
    if (!origin_ok())     { server.send(403, "text/plain", "Forbidden"); return; }
    if (vfd_web_locked()) { server.send(403, "text/plain", "VFD control locked"); return; }
    bool ok = vfd_run();
    server.send(ok ? 200 : 502, "text/plain", ok ? "Forward command sent" : "VFD comms error");
}

static void handle_vfd_reverse()
{
    if (!origin_ok())     { server.send(403, "text/plain", "Forbidden"); return; }
    if (vfd_web_locked()) { server.send(403, "text/plain", "VFD control locked"); return; }
    bool ok = vfd_reverse();
    server.send(ok ? 200 : 502, "text/plain", ok ? "Reverse command sent" : "VFD comms error");
}

/* Deliberately not gated by origin_ok() or vfd_web_locked(): Stop must always
   go through, from any source, lockout engaged or not. Never make the safe
   direction harder to reach than the dangerous one. */
static void handle_vfd_stop()
{
    bool ok = vfd_stop();
    server.send(ok ? 200 : 502, "text/plain", ok ? "Stop command sent" : "VFD comms error");
}

static void handle_vfd_reset()
{
    if (!origin_ok())     { server.send(403, "text/plain", "Forbidden"); return; }
    if (vfd_web_locked()) { server.send(403, "text/plain", "VFD control locked"); return; }
    bool ok = vfd_reset_fault();
    server.send(ok ? 200 : 502, "text/plain", ok ? "Fault reset sent" : "VFD comms error");
}

static void handle_vfd_freq()
{
    if (!origin_ok())     { server.send(403, "text/plain", "Forbidden"); return; }
    if (vfd_web_locked()) { server.send(403, "text/plain", "VFD control locked"); return; }
    if (!server.hasArg("hz")) {
        server.send(400, "text/plain", "Missing hz parameter");
        return;
    }
    int val = server.arg("hz").toInt();
    if (val < 0 || val > 40000) {
        server.send(400, "text/plain", "hz must be 0–40000 (0.01 Hz units)");
        return;
    }
    bool ok = vfd_set_freq((uint16_t)val);
    char buf[32];
    if (ok) snprintf(buf, sizeof(buf), "Freq set: %.2f Hz", val / 100.0f);
    else    snprintf(buf, sizeof(buf), "VFD comms error");
    server.send(ok ? 200 : 502, "text/plain", buf);
}

static void handle_vfd_rpm()
{
    if (!origin_ok())     { server.send(403, "text/plain", "Forbidden"); return; }
    if (vfd_web_locked()) { server.send(403, "text/plain", "VFD control locked"); return; }
    if (!server.hasArg("rpm")) {
        server.send(400, "text/plain", "Missing rpm parameter");
        return;
    }
    int rpm_val = server.arg("rpm").toInt();
    if (rpm_val < 0) {
        server.send(400, "text/plain", "rpm must be >= 0");
        return;
    }
    /* Convert RPM → frequency in 0.01 Hz units using the scaling curve:
       hz = rpm × (vfd_base_hz / vfd_base_rpm)
       Use 64-bit intermediate to avoid overflow at high RPM × high base_hz values. */
    uint32_t hz_cents = (uint32_t)(((uint64_t)rpm_val * vfd_base_hz) / vfd_base_rpm);

    /* Cap at the configured max frequency; back-calculate the actual RPM applied. */
    bool clamped = false;
    if (hz_cents > vfd_max_hz) {
        hz_cents = vfd_max_hz;
        clamped  = true;
    }
    int actual_rpm = (int)(((uint64_t)hz_cents * vfd_base_rpm) / vfd_base_hz);

    bool ok = vfd_set_freq((uint16_t)hz_cents);
    char buf[80];
    if (!ok) {
        snprintf(buf, sizeof(buf), "VFD comms error");
        server.send(502, "text/plain", buf);
    } else if (clamped) {
        snprintf(buf, sizeof(buf),
                 "Input exceeded max \u2014 speed set to %d RPM (%.2f Hz)",
                 actual_rpm, hz_cents / 100.0f);
        server.send(200, "text/plain", buf);
    } else {
        snprintf(buf, sizeof(buf), "Speed set: %d RPM (%.2f Hz)",
                 actual_rpm, hz_cents / 100.0f);
        server.send(200, "text/plain", buf);
    }
}

static void handle_vfd_settings()
{
    if (!origin_ok())     { server.send(403, "text/plain", "Forbidden"); return; }
    if (vfd_web_locked()) { server.send(403, "text/plain", "VFD control locked"); return; }
    if (!server.hasArg("slave") || !server.hasArg("maxhz") ||
        !server.hasArg("basehz") || !server.hasArg("baserpm")) {
        server.send(400, "text/plain", "Missing parameter");
        return;
    }
    int s  = server.arg("slave").toInt();
    int m  = server.arg("maxhz").toInt();
    int bh = server.arg("basehz").toInt();
    int br = server.arg("baserpm").toInt();
    if (s < 1 || s > 31) {
        server.send(400, "text/plain", "slave must be 1–31");
        return;
    }
    if (m < 1 || m > 40000) {
        server.send(400, "text/plain", "maxhz must be 1–40000 (0.01 Hz units)");
        return;
    }
    if (bh < 1 || bh > 40000) {
        server.send(400, "text/plain", "basehz must be 1–40000 (0.01 Hz units)");
        return;
    }
    if (br < 1 || br > 60000) {
        server.send(400, "text/plain", "baserpm must be 1–60000");
        return;
    }
    vfd_slave    = (uint8_t)s;
    vfd_max_hz   = (uint16_t)m;
    vfd_base_hz  = (uint16_t)bh;
    vfd_base_rpm = (uint16_t)br;
    char buf[80];
    snprintf(buf, sizeof(buf), "Saved: slave=%d, max=%.2f Hz, base=%.2f Hz @ %d RPM",
             s, m / 100.0f, bh / 100.0f, br);
    server.send(200, "text/plain", buf);
    save_config();
}

static void handle_vfd_status()
{
    /* Force a fresh poll (rather than waiting for loop()'s next scheduled
       one) so a web client always sees current state — but through the same
       vfd_poll() that owns all VFD state, not a second, independent read.
       This is the VFD driver's swap boundary (see the "VFD DRIVER
       ABSTRACTION" section above vfd_init()): nothing outside that section
       should touch the bus or decode protocol-specific bits itself. */
    vfd_poll();

    /* Pot is considered active if the ADC reads meaningfully above zero,
       meaning a potentiometer is wired to the pin and controlling speed. */
    bool pot_active = (analogRead(VFD_POT_PIN) > 50);

    bool web_lock = vfd_web_locked();

    char buf[240];
    snprintf(buf, sizeof(buf),
             "{\"comms_ok\":%s,\"running\":%s,\"reverse\":%s,\"status\":%u,\"fault\":%u,"
             "\"freq\":%u,\"output_freq\":%u,\"base_hz\":%u,\"base_rpm\":%u,"
             "\"pot_active\":%s,\"web_lock\":%s}",
             vfd_comms_ok       ? "true" : "false",
             vfd_running        ? "true" : "false",
             vfd_reverse_active ? "true" : "false",
             (unsigned)vfd_status_word,
             (unsigned)vfd_fault_code,
             (unsigned)vfd_freq_ref,
             (unsigned)vfd_output_freq,
             (unsigned)vfd_base_hz,
             (unsigned)vfd_base_rpm,
             pot_active         ? "true" : "false",
             web_lock           ? "true" : "false");
    server.send(200, "application/json", buf);
}

/* ---- End VFD web handlers ---- */

static void show_overlay(const char *text, uint32_t duration_ms)
{
    lv_label_set_text(ip_overlay, text);
    lv_obj_t *box = lv_obj_get_parent(ip_overlay);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_HIDDEN);
    g_ip_show_until = duration_ms ? (millis() + duration_ms) : 0;
}

static void handle_wifi_scan()
{
    /* In AP-only mode the radio can't scan; switch to AP+STA for the duration. */
    bool was_ap_only = (wifi_mode == MODE_AP);
    if (was_ap_only) WiFi.mode(WIFI_AP_STA);

    int n = WiFi.scanNetworks();   /* blocking scan — typically 2–4 s */

    if (was_ap_only) WiFi.mode(WIFI_AP);   /* restore */

    String json = "[";
    if (n > 0) {
        for (int i = 0; i < n; i++) {
            if (i > 0) json += ",";
            String ssid = WiFi.SSID(i);
            ssid.replace("\"", "\\\"");
            json += "\"" + ssid + "\"";
        }
    }
    json += "]";
    WiFi.scanDelete();
    server.send(200, "application/json", json);
}

static void handle_set_wifi()
{
    if (!origin_ok()) { server.send(403, "text/plain", "Forbidden"); return; }
    if (!server.hasArg("mode")) {
        server.send(400, "text/plain", "Missing mode");
        return;
    }
    String mode = server.arg("mode");
    if (mode == "ap") {
        wifi_mode = MODE_AP;
        server.send(200, "text/plain", "Switching to AP mode — connect to " WIFI_AP_SSID);
    } else if (mode == "sta") {
        if (!server.hasArg("ssid") || server.arg("ssid").length() == 0) {
            server.send(400, "text/plain", "SSID required for station mode");
            return;
        }
        sta_ssid     = server.arg("ssid");
        sta_password = server.hasArg("pass") ? server.arg("pass") : "";
        wifi_mode    = MODE_STA;
        server.send(200, "text/plain", "Connecting to " + sta_ssid + "...");
    } else {
        server.send(400, "text/plain", "Unknown mode");
        return;
    }
    /* Set flag — loop() will call apply_wifi_config() after handleClient() returns */
    wifi_reconfig_pending = true;
}

/* Helper: pump LVGL for ms milliseconds */
static void lvgl_delay(uint32_t ms)
{
    uint32_t t      = millis();
    uint32_t last_lv = t;
    while (millis() - t < ms) {
        uint32_t now2 = millis();
        lv_tick_inc(now2 - last_lv);
        last_lv = now2;
        lv_timer_handler();
        delay(10);
    }
}

/* Tears down whatever radio state exists and brings up wifi_mode (STA or AP),
   blocking for up to ~20s on a STA connection attempt before falling back to
   AP. Called from loop() (after a /set-wifi POST sets wifi_reconfig_pending,
   or after the long-press-to-AP button gesture) and once from setup() on
   boot — never called from inside a web handler directly, since it blocks
   long enough to want handleClient() to have already returned first. */
static void apply_wifi_config()
{
    server.stop();
    dnsServer.stop();
    WiFi.disconnect(false);       /* disconnect without powering down radio */
    WiFi.softAPdisconnect(true);

    if (wifi_mode == MODE_STA) {
        show_overlay(("Connecting to " + sta_ssid).c_str(), 0);
        lvgl_delay(100);          /* let overlay render before radio init blocks */

        WiFi.mode(WIFI_STA);
        lvgl_delay(200);          /* settle after mode switch */
        WiFi.begin(sta_ssid.c_str(), sta_password.c_str());

        /* Wait up to 20 s, pumping LVGL so the display stays alive */
        uint32_t t       = millis();
        uint32_t last_lv = t;
        while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) {
            uint32_t now2 = millis();
            lv_tick_inc(now2 - last_lv);
            last_lv = now2;
            lv_timer_handler();
            delay(100);
        }

        if (WiFi.status() == WL_CONNECTED) {
            current_ip = WiFi.localIP().toString();
            current_wifi_mode = MODE_STA;
            show_overlay((sta_ssid + "\n" + current_ip).c_str(), 5000);
        } else {
            /* STA failed — fall back to AP visually but DO NOT change wifi_mode
               so saved config still has STA and will retry on next reboot.
               current_wifi_mode DOES change, so the UI reflects the live radio state. */
            show_overlay(("Failed: " + sta_ssid + "\n" + WIFI_AP_SSID).c_str(), 4000);
            lvgl_delay(500);

            IPAddress local(192, 168, 4, 1);
            WiFi.mode(WIFI_AP);
            WiFi.softAPConfig(local, local, IPAddress(255, 255, 255, 0));
            WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
            current_ip = "192.168.4.1";
            current_wifi_mode = MODE_AP;
            dnsServer.start(53, "*", local);
            server.begin();
            return;   /* return early — don't save, preserving STA config */
        }
    }

    if (wifi_mode == MODE_AP) {
        show_overlay("Starting AP...", 0);
        lvgl_delay(100);

        IPAddress local(192, 168, 4, 1);
        WiFi.mode(WIFI_AP);
        WiFi.softAPConfig(local, local, IPAddress(255, 255, 255, 0));
        WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
        current_ip = "192.168.4.1";
        current_wifi_mode = MODE_AP;
        dnsServer.start(53, "*", local);

        /* duration_ms=0 → stays visible until explicitly hidden or overwritten;
           the overlay will be on screen when loop() starts and cleared on next button press */
        show_overlay((String(WIFI_AP_SSID) + "\n192.168.4.1").c_str(), 0);
        lvgl_delay(200);    /* one render pass so the frame is actually pushed to the display */
    }

    server.begin();
    save_config();   /* only reached on successful connect or explicit AP switch */
}

/* ---- Persistent config (EEPROM) ----
   Layout (byte offsets):
     0        : magic byte (0xB0 = valid data present)
     1        : wifi_mode (0 = AP, 1 = STA)
     2–65     : sta_ssid  (null-terminated, max 63 chars)
     66–129   : sta_password (null-terminated, max 63 chars)
     130–131  : hob_threads    (uint16_t, 2 bytes)
     132–133  : gear_teeth     (uint16_t, 2 bytes)
     134      : encoder_reversed (0 or 1)
     135–136  : encoder_ppr    (uint16_t, 2 bytes)
     137–138  : pulley_driver  (uint16_t, 2 bytes)
     139–140  : pulley_driven  (uint16_t, 2 bytes)
     141      : vfd_slave      (uint8_t)
     142–143  : vfd_max_hz     (uint16_t, 0.01 Hz units)
     144–145  : vfd_base_hz    (uint16_t, 0.01 Hz units)
     146–147  : vfd_base_rpm   (uint16_t)                */
#define EE_MAGIC_ADDR      0
#define EE_MODE_ADDR       1
#define EE_SSID_ADDR       2
#define EE_PASS_ADDR       66
#define EE_HOB_ADDR        130   /* hob_threads uint16_t */
#define EE_TEETH_ADDR      132   /* gear_teeth  uint16_t */
#define EE_REV_ADDR        134
#define EE_PPR_ADDR        135
#define EE_PULLEY_DRV_ADDR 137   /* pulley_driver uint16_t */
#define EE_PULLEY_DRN_ADDR 139   /* pulley_driven uint16_t */
#define EE_VFD_SLAVE_ADDR  141   /* vfd_slave    uint8_t  */
#define EE_VFD_MAXHZ_ADDR  142   /* vfd_max_hz   uint16_t */
#define EE_VFD_BASEHZ_ADDR 144   /* vfd_base_hz  uint16_t */
#define EE_VFD_BASERPM_ADDR 146  /* vfd_base_rpm uint16_t */
#define EE_TOTAL_SIZE      148
#define EE_MAGIC_VAL       0xB0   /* bumped from 0xAF — forces re-init on first boot */

static void save_config()
{
    EEPROM.begin(EE_TOTAL_SIZE);
    EEPROM.write(EE_MAGIC_ADDR, EE_MAGIC_VAL);
    EEPROM.write(EE_MODE_ADDR,  (uint8_t)wifi_mode);

    /* Write SSID — zero-pad the full field so stale data can't bleed through */
    for (int i = 0; i < 64; i++)
        EEPROM.write(EE_SSID_ADDR + i,
                     i < (int)sta_ssid.length() ? sta_ssid[i] : 0);

    /* Write password */
    for (int i = 0; i < 64; i++)
        EEPROM.write(EE_PASS_ADDR + i,
                     i < (int)sta_password.length() ? sta_password[i] : 0);

    /* Write hob_threads and gear_teeth as uint16_t (little-endian) */
    uint16_t hob = (uint16_t)hob_threads;
    uint16_t tth = (uint16_t)gear_teeth;
    EEPROM.write(EE_HOB_ADDR,     (uint8_t)(hob & 0xFF));
    EEPROM.write(EE_HOB_ADDR + 1, (uint8_t)(hob >> 8));
    EEPROM.write(EE_TEETH_ADDR,     (uint8_t)(tth & 0xFF));
    EEPROM.write(EE_TEETH_ADDR + 1, (uint8_t)(tth >> 8));

    /* Write encoder_reversed */
    EEPROM.write(EE_REV_ADDR, encoder_reversed ? 1 : 0);

    /* Write encoder_ppr as uint16_t (little-endian) */
    uint16_t ppr = (uint16_t)encoder_ppr;
    EEPROM.write(EE_PPR_ADDR,     (uint8_t)(ppr & 0xFF));
    EEPROM.write(EE_PPR_ADDR + 1, (uint8_t)(ppr >> 8));

    /* Write pulley_driver and pulley_driven as uint16_t (little-endian) */
    uint16_t pdrv = (uint16_t)pulley_driver;
    uint16_t pdrn = (uint16_t)pulley_driven;
    EEPROM.write(EE_PULLEY_DRV_ADDR,     (uint8_t)(pdrv & 0xFF));
    EEPROM.write(EE_PULLEY_DRV_ADDR + 1, (uint8_t)(pdrv >> 8));
    EEPROM.write(EE_PULLEY_DRN_ADDR,     (uint8_t)(pdrn & 0xFF));
    EEPROM.write(EE_PULLEY_DRN_ADDR + 1, (uint8_t)(pdrn >> 8));

    /* Write VFD slave address, max frequency, baseline frequency and RPM */
    EEPROM.write(EE_VFD_SLAVE_ADDR, vfd_slave);
    uint16_t vmaxhz = vfd_max_hz;
    EEPROM.write(EE_VFD_MAXHZ_ADDR,     (uint8_t)(vmaxhz & 0xFF));
    EEPROM.write(EE_VFD_MAXHZ_ADDR + 1, (uint8_t)(vmaxhz >> 8));
    uint16_t vbasehz = vfd_base_hz;
    EEPROM.write(EE_VFD_BASEHZ_ADDR,     (uint8_t)(vbasehz & 0xFF));
    EEPROM.write(EE_VFD_BASEHZ_ADDR + 1, (uint8_t)(vbasehz >> 8));
    uint16_t vbaserpm = vfd_base_rpm;
    EEPROM.write(EE_VFD_BASERPM_ADDR,     (uint8_t)(vbaserpm & 0xFF));
    EEPROM.write(EE_VFD_BASERPM_ADDR + 1, (uint8_t)(vbaserpm >> 8));

    EEPROM.commit();
    EEPROM.end();
}

static void load_config()
{
    EEPROM.begin(EE_TOTAL_SIZE);

    if (EEPROM.read(EE_MAGIC_ADDR) != EE_MAGIC_VAL) {
        /* No valid data or layout changed — stay with defaults */
        EEPROM.end();
        return;
    }

    wifi_mode = (EEPROM.read(EE_MODE_ADDR) == 1) ? MODE_STA : MODE_AP;

    /* Read SSID */
    char buf[64];
    for (int i = 0; i < 63; i++) buf[i] = (char)EEPROM.read(EE_SSID_ADDR + i);
    buf[63] = '\0';
    sta_ssid = String(buf);

    /* Read password */
    for (int i = 0; i < 63; i++) buf[i] = (char)EEPROM.read(EE_PASS_ADDR + i);
    buf[63] = '\0';
    sta_password = String(buf);

    /* Read hob_threads and gear_teeth */
    uint16_t hob_load = (uint16_t)EEPROM.read(EE_HOB_ADDR)
                      | ((uint16_t)EEPROM.read(EE_HOB_ADDR + 1) << 8);
    uint16_t tth_load = (uint16_t)EEPROM.read(EE_TEETH_ADDR)
                      | ((uint16_t)EEPROM.read(EE_TEETH_ADDR + 1) << 8);
    if (hob_load >= 1 && hob_load <= 9999)
        hob_threads = hob_load;
    if (tth_load >= 1 && tth_load <= 9999)
        gear_teeth = tth_load;

    /* Read encoder_reversed */
    encoder_reversed = (EEPROM.read(EE_REV_ADDR) == 1);

    /* Read encoder_ppr */
    uint16_t ppr = (uint16_t)EEPROM.read(EE_PPR_ADDR)
                 | ((uint16_t)EEPROM.read(EE_PPR_ADDR + 1) << 8);
    if (ppr >= 1 && ppr <= 10000)
        encoder_ppr = ppr;

    /* Read pulley_driver and pulley_driven */
    uint16_t pdrv_load = (uint16_t)EEPROM.read(EE_PULLEY_DRV_ADDR)
                       | ((uint16_t)EEPROM.read(EE_PULLEY_DRV_ADDR + 1) << 8);
    uint16_t pdrn_load = (uint16_t)EEPROM.read(EE_PULLEY_DRN_ADDR)
                       | ((uint16_t)EEPROM.read(EE_PULLEY_DRN_ADDR + 1) << 8);
    if (pdrv_load >= 1 && pdrv_load <= 9999)
        pulley_driver = pdrv_load;
    if (pdrn_load >= 1 && pdrn_load <= 9999)
        pulley_driven = pdrn_load;

    /* Read VFD slave address, max frequency, baseline frequency and RPM */
    uint8_t vfd_slave_load = EEPROM.read(EE_VFD_SLAVE_ADDR);
    if (vfd_slave_load >= 1 && vfd_slave_load <= 31)
        vfd_slave = vfd_slave_load;

    uint16_t vfd_maxhz_load = (uint16_t)EEPROM.read(EE_VFD_MAXHZ_ADDR)
                            | ((uint16_t)EEPROM.read(EE_VFD_MAXHZ_ADDR + 1) << 8);
    if (vfd_maxhz_load >= 1 && vfd_maxhz_load <= 40000)
        vfd_max_hz = vfd_maxhz_load;

    uint16_t vfd_basehz_load = (uint16_t)EEPROM.read(EE_VFD_BASEHZ_ADDR)
                             | ((uint16_t)EEPROM.read(EE_VFD_BASEHZ_ADDR + 1) << 8);
    if (vfd_basehz_load >= 1 && vfd_basehz_load <= 40000)
        vfd_base_hz = vfd_basehz_load;

    uint16_t vfd_baserpm_load = (uint16_t)EEPROM.read(EE_VFD_BASERPM_ADDR)
                              | ((uint16_t)EEPROM.read(EE_VFD_BASERPM_ADDR + 1) << 8);
    if (vfd_baserpm_load >= 1 && vfd_baserpm_load <= 60000)
        vfd_base_rpm = vfd_baserpm_load;

    EEPROM.end();
}

/* Loads saved settings, registers every HTTP route, then brings up the radio
   via apply_wifi_config(). Called once from setup(). */
static void setup_wifi()
{
    load_config();
    server.collectHeaders("Origin");   /* needed for origin_ok()'s CSRF check */
    server.on("/",             HTTP_GET,  handle_root);
    server.on("/set",          HTTP_POST, handle_set);
    server.on("/set-encoder",  HTTP_POST, handle_set_encoder);
    server.on("/set-ppr",      HTTP_POST, handle_set_ppr);
    server.on("/set-pulley",   HTTP_POST, handle_set_pulley);
    server.on("/set-wifi",     HTTP_POST, handle_set_wifi);
    server.on("/wifi-scan",    HTTP_GET,  handle_wifi_scan);
    server.on("/vfd-rpm",      HTTP_POST, handle_vfd_rpm);
    server.on("/vfd-run",      HTTP_POST, handle_vfd_run);
    server.on("/vfd-reverse",  HTTP_POST, handle_vfd_reverse);
    server.on("/vfd-stop",     HTTP_POST, handle_vfd_stop);
    server.on("/vfd-reset",    HTTP_POST, handle_vfd_reset);
    server.on("/vfd-freq",     HTTP_POST, handle_vfd_freq);
    server.on("/vfd-settings", HTTP_POST, handle_vfd_settings);
    server.on("/vfd-status",   HTTP_GET,  handle_vfd_status);
    /* Captive portal: redirect any unrecognised URL to the root page */
    server.onNotFound([]() {
        server.sendHeader("Location", "/", true);
        server.send(302, "text/plain", "");
    });
    apply_wifi_config();
}

void setup()
{
    Serial.begin(115200);

    pinMode(PIN_LED, OUTPUT);

    /* Encoder — interrupt on rising edge of channel A */
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_isr, RISING);

    /* IP button */
    pinMode(BTN_PIN, INPUT_PULLUP);

    /* VFD physical controls */
    pinMode(VFD_BTN_START,   INPUT_PULLUP);
    pinMode(VFD_BTN_REVERSE, INPUT_PULLUP);
    pinMode(VFD_BTN_STOP,    INPUT_PULLUP);
    pinMode(VFD_POT_PIN,     INPUT);   /* ADC — no pullup */
    analogReadResolution(12);          /* Force 12-bit ADC (0–4095); default is 10-bit */

    /* RPM source select button */
    pinMode(RPM_SRC_BTN,  INPUT_PULLUP);
    pinMode(VFD_LOCK_BTN, INPUT_PULLUP);

    /* Initialise display */
    tft.begin();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);

    /* Initialise LVGL 8 */
    lv_init();

    /* Register display driver (LVGL 8 API) */
    static lv_disp_draw_buf_t draw_buf_dsc;
    lv_disp_draw_buf_init(&draw_buf_dsc, draw_buf_1, nullptr,
                          DISP_HOR_RES * DRAW_BUF_ROWS);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res   = DISP_HOR_RES;
    disp_drv.ver_res   = DISP_VER_RES;
    disp_drv.flush_cb  = disp_flush;
    disp_drv.draw_buf  = &draw_buf_dsc;
    lv_disp_drv_register(&disp_drv);

    create_tachometer();
    run_startup_animation();

    /* ---- Initialise stepper driver ---- */
    stepper_init();

    /* ---- Initialise VFD serial port ---- */
    vfd_init();

    /* Load vfd_slave (and other settings) before commanding the drive, so the
       safety stop below targets the correct Modbus address. */
    load_config();

    /* Safety: force the drive to a known-stopped state on every boot/reset.
       The drive has no idea "the controller just restarted" — if it was left
       running before a crash, reflash, or power blip, it keeps spinning right
       through the reboot unless we explicitly tell it to stop. Retry briefly
       in case the RS-485 bus / drive isn't ready the instant we power up. */
    {
        bool stopped = false;
        for (uint8_t attempt = 0; attempt < 5 && !stopped; attempt++) {
            stopped = vfd_stop();
            if (!stopped) delay(200);
        }
        Serial.println(stopped
            ? "[VFD] boot safety stop OK"
            : "[VFD] boot safety stop FAILED — no comms yet, will retry via vfd_poll()");
    }

    /* ---- Start WiFi AP and web server ---- */
    setup_wifi();
}

/* Main loop, run once per Arduino core tick. Responsibilities, in order:
   IP button / long-press-to-AP state machine, encoder → RPM → stepper speed
   (also drives the tachometer needle/readout), LVGL rendering, the three VFD
   physical buttons, the direction arrows, the speed potentiometer, the VFD
   status poll, the onboard status LED, and the web server / DNS / pending
   WiFi reconfig. Nothing here blocks for long except apply_wifi_config()
   (button long-press or a pending web request), which pumps LVGL itself
   while it waits so the display doesn't freeze during a WiFi (re)connect. */
void loop()
{
    /* Drive LVGL tick from millis() */
    static uint32_t last_tick_ms = 0;
    static bool     btn_last     = HIGH;
    static uint32_t btn_down_at  = 0;      /* millis() when button was pressed */
    static bool     long_fired   = false;  /* long-press action already triggered */
    uint32_t now = millis();
    lv_tick_inc(now - last_tick_ms);
    last_tick_ms = now;

    /* ---- Button state machine ----
       Short press (< 2 s): show IP for 5 s
       Long press (>= 2 s): switch to AP mode            */
    const uint32_t LONG_PRESS_MS = 2000;

    bool btn_now = digitalRead(BTN_PIN);

    if (btn_last == HIGH && btn_now == LOW) {
        /* Falling edge — button just pressed */
        btn_down_at = now;
        long_fired  = false;
    }

    if (btn_now == LOW && !long_fired &&
        btn_down_at && (now - btn_down_at >= LONG_PRESS_MS))
    {
        /* Long press: switch to AP — apply_wifi_config() will show notifications */
        long_fired = true;
        wifi_mode  = MODE_AP;
        apply_wifi_config();
    }

    if (btn_last == LOW && btn_now == HIGH) {
        /* Rising edge — button released */
        if (!long_fired)
            show_overlay(current_ip.c_str(), 5000);  /* short press: show IP */
        btn_down_at = 0;
    }

    btn_last = btn_now;

    /* Hide overlay once the display window expires */
    if (g_ip_show_until && now >= g_ip_show_until) {
        lv_obj_add_flag(lv_obj_get_parent(ip_overlay), LV_OBJ_FLAG_HIDDEN);
        g_ip_show_until = 0;
    }

    /* ---- RPM source select ---- */
    bool vfd_rpm_source = (digitalRead(RPM_SRC_BTN) == LOW);

    static uint32_t last_rpm_ms  = 0;
    static int32_t  last_pos     = 0;
    static int32_t  smooth_rpm   = 0;   /* signed, exponentially smoothed RPM (encoder) */
    static bool     last_vfd_src = false;

    if (now - last_rpm_ms >= RPM_UPDATE_MS)
    {
        uint32_t elapsed_ms = now - last_rpm_ms;
        last_rpm_ms = now;

        /* Always keep the encoder state current so stepper tracking stays accurate */
        noInterrupts();
        int32_t current_pos = encoder_pos;
        interrupts();

        int32_t delta   = current_pos - last_pos;
        last_pos        = current_pos;

        int32_t raw_rpm = (delta * 60000L) / ((int32_t)encoder_ppr * (int32_t)elapsed_ms);
        if (raw_rpm >  (int32_t)METER_MAX_RPM) raw_rpm =  (int32_t)METER_MAX_RPM;
        if (raw_rpm < -(int32_t)METER_MAX_RPM) raw_rpm = -(int32_t)METER_MAX_RPM;

        smooth_rpm = (smooth_rpm * 6 + raw_rpm * 4) / 10;

        /* Stepper always tracks encoder regardless of display source */
        set_stepper_rpm(smooth_rpm);

        /* Choose display RPM based on source button */
        uint32_t display_rpm;
        if (vfd_rpm_source) {
            /* VFD source: convert output_freq (0.01 Hz units) → RPM */
            display_rpm = (vfd_base_hz > 0)
                ? (uint32_t)(((uint32_t)vfd_output_freq * vfd_base_rpm) / vfd_base_hz)
                : 0;
        } else {
            display_rpm = (uint32_t)abs(smooth_rpm);
        }
        if (display_rpm > METER_MAX_RPM) display_rpm = METER_MAX_RPM;

        /* Show source label when it changes */
        if (vfd_rpm_source != last_vfd_src) {
            show_overlay(vfd_rpm_source ? "RPM: VFD" : "RPM: Encoder", 2000);
            last_vfd_src = vfd_rpm_source;
        }

        /* Scale needle to 0–400 range (scale units = RPM / 10) */
        lv_meter_set_indicator_end_value(meter, needle, (int32_t)(display_rpm / 10));

        /* Update live RPM readout */
        char buf[8];
        snprintf(buf, sizeof(buf), "%04lu", display_rpm);
        lv_label_set_text(rpm_readout, buf);
    }

    lv_timer_handler();

    /* ---- VFD physical controls — each button sends exactly one command ---- */

    /* Start — always sends run forward */
    static bool vfd_start_last = HIGH;
    bool vfd_start_now = digitalRead(VFD_BTN_START);
    if (vfd_start_last == HIGH && vfd_start_now == LOW) vfd_run();
    vfd_start_last = vfd_start_now;

    /* Reverse — always sends run reverse */
    static bool vfd_rev_last = HIGH;
    bool vfd_rev_now = digitalRead(VFD_BTN_REVERSE);
    if (vfd_rev_last == HIGH && vfd_rev_now == LOW) vfd_reverse();
    vfd_rev_last = vfd_rev_now;

    /* Stop — always sends stop */
    static bool vfd_stop_last = HIGH;
    bool vfd_stop_now = digitalRead(VFD_BTN_STOP);
    if (vfd_stop_last == HIGH && vfd_stop_now == LOW) vfd_stop();
    vfd_stop_last = vfd_stop_now;

    update_direction_arrows();

    /* Potentiometer — read ADC, map to RPM, send on meaningful change */
    static uint32_t last_pot_ms  = 0;
    static int32_t  last_pot_raw = -1;   /* -1 forces a send on first iteration */
    if (now - last_pot_ms >= 50) {       /* sample every 50 ms */
        last_pot_ms = now;
        int32_t raw = analogRead(VFD_POT_PIN);   /* 0–4095 (12-bit) */
        if (abs(raw - last_pot_raw) > VFD_POT_DEADBAND) {
            last_pot_raw = raw;
            /* Map POT_MIN–4095 → 0–vfd_max_hz, clamping below min to zero */
            uint32_t hz_cents = 0;
            if (raw > VFD_POT_MIN) {
                hz_cents = ((uint32_t)(raw - VFD_POT_MIN) * vfd_max_hz)
                           / (4095 - VFD_POT_MIN);
            }
            vfd_set_freq((uint16_t)hz_cents);
        }
    }

    /* ---- Poll VFD status ---- */
    static uint32_t last_vfd_ms = 0;
    if (now - last_vfd_ms >= VFD_POLL_MS) {
        last_vfd_ms = now;
        vfd_poll();
        update_direction_arrows();
    }

    /* ---- Onboard LED: 1 Hz when VFD running, 0.25 Hz when stopped ---- */
    static uint32_t last_led_ms = 0;
    static bool     led_state   = false;
    uint32_t blink_ms = vfd_running ? 500 : 2000;
    if (now - last_led_ms >= blink_ms) {
        last_led_ms = now;
        led_state   = !led_state;
        digitalWrite(PIN_LED, led_state);
    }

    /* Service web requests */
    server.handleClient();
    dnsServer.processNextRequest();

    /* Apply any pending WiFi reconfiguration requested by the web handler */
    if (wifi_reconfig_pending) {
        wifi_reconfig_pending = false;
        apply_wifi_config();
    }

    delay(5);
}
