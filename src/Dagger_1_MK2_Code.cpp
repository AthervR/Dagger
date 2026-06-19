/**
 * Flight Computer Hardware Bring-Up Sketch
 * Teensy 4.1
 *
 * Devices:
 *   ICM-20948  9-DOF IMU     — SPI0,  CS pin 10, INT pin 8
 *   BMP390     Barometer     — SPI0,  CS pin 9  (shares bus with IMU), INT pin 7 (optional)
 *   SSD1306    0.91" OLED    — I2C0,  address 0x3C
 *   NEO-M9N    GPS           — UART1, pins 0(RX) / 1(TX), PPS pin 2
 *   microSD    Flight log    — SDIO   (Teensy 4.1 built-in slot, no GPIO cost)
 *
 * Required libraries (Arduino Library Manager):
 *   "SparkFun 9DoF IMU Breakout - ICM 20948"   by SparkFun Electronics
 *   "Adafruit BMP3XX Library"                   by Adafruit  (+BusIO dependency)
 *   "Adafruit SSD1306"                          by Adafruit  (+GFX dependency)
 *   Teensy SD (bundled with Teensyduino — do NOT install a separate SD library)
 *
 * Note on "Multiple libraries found for SD.h":
 *   This warning is benign. Arduino IDE correctly uses the Teensy-bundled SD
 *   library (packages/teensy/.../SD) which is the only one that supports
 *   BUILTIN_SDCARD and the Teensy 4.1 SDIO interface. The others are ignored.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "ICM_20948.h"          // SparkFun 9DoF IMU Breakout - ICM 20948
#include <Adafruit_BMP3XX.h>    // Adafruit BMP3XX
#include <Adafruit_SSD1306.h>   // Adafruit SSD1306
#include <SD.h>                 // Teensy-bundled SD (SDIO)
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  // SparkFun u-blox GNSS v2

// =============================================================================
// Pin assignments
// =============================================================================
constexpr uint8_t PIN_CS_IMU  = 10;   // SPI chip-select: ICM-20948
constexpr uint8_t PIN_CS_BARO =  9;   // SPI chip-select: BMP390
constexpr uint8_t PIN_INT_IMU =  8;   // Data-ready interrupt from ICM-20948
constexpr uint8_t PIN_INT_BARO=  7;   // Data-ready interrupt from BMP390 (optional)
constexpr uint8_t PIN_GPS_PPS =  2;   // GPS 1 PPS — latched with micros() for EKF time sync

// =============================================================================
// Peripheral objects
// =============================================================================
ICM_20948_SPI    imu;
Adafruit_BMP3XX  baro;
Adafruit_SSD1306 oled(128, 32, &Wire);   // 0.91" SSD1306, I2C0
SFE_UBLOX_GNSS   gnss;                   // NEO-M9N over UART1

// Component status flags — set during setup(), read in loop() and OLED update
bool imu_ok  = false;
bool baro_ok = false;
bool oled_ok = false;
bool sd_ok   = false;
bool gps_ok  = false;

// =============================================================================
// IMU data-ready interrupt
// =============================================================================
volatile bool imu_data_ready = false;
void imu_isr() { imu_data_ready = true; }

// =============================================================================
// GPS PPS interrupt — latches MCU microsecond counter on each rising edge.
// One pulse per second (after fix). EKF uses this to correlate MCU time with
// GPS time-of-week from NAV-PVT iTOW field.
// =============================================================================
volatile uint32_t pps_timestamp_us = 0;
void pps_isr() { pps_timestamp_us = micros(); }

// =============================================================================
// GPS state — populated by pvt_callback()
// =============================================================================
struct GpsState {
  double  lat    = 0.0;    // degrees
  double  lon    = 0.0;    // degrees
  float   alt_m  = 0.0f;   // metres MSL
  float   spd_ms = 0.0f;   // ground speed m/s
  uint8_t fix    = 0;      // 0=no fix, 2=2D, 3=3D
  uint8_t sats   = 0;      // satellites used
  bool    fresh  = false;  // cleared by consumer after reading
} gps;

// =============================================================================
// Forward declarations
// =============================================================================
void pvt_callback(UBX_NAV_PVT_data_t d);
void pps_isr();

// =============================================================================
// GPS PVT callback — fired by gnss.checkCallbacks() on each new NAV-PVT packet
// =============================================================================
void pvt_callback(UBX_NAV_PVT_data_t d) {
  gps.fix    = d.fixType;
  gps.sats   = d.numSV;
  gps.lat    = d.lat    * 1e-7;
  gps.lon    = d.lon    * 1e-7;
  gps.alt_m  = d.hMSL  * 0.001f;
  gps.spd_ms = d.gSpeed * 0.001f;
  gps.fresh  = true;
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // solid ON during boot

  Serial.begin(115200);
  // Wait up to 3 s for USB serial monitor.
  // REMOVE for flight firmware — stalls boot if no PC is connected.
  while (!Serial && millis() < 3000);
  Serial.println("\n[BOOT] Flight computer bring-up starting");

  // SPI0 — Sensor bus
  SPI.begin();
  pinMode(PIN_CS_IMU,  OUTPUT); digitalWrite(PIN_CS_IMU,  HIGH);
  pinMode(PIN_CS_BARO, OUTPUT); digitalWrite(PIN_CS_BARO, HIGH);

  imu.begin(PIN_CS_IMU, SPI, 4000000);   // 4 MHz — conservative for bring-up (max is 7 MHz)
  imu_ok = (imu.status == ICM_20948_Stat_Ok);
  if (!imu_ok) {
    Serial.println("[IMU ] FAIL — verify MOSI(11)/MISO(12)/SCK(13)/CS(10) and 3.3V");
  } else {
    imu.sleep(true);
    imu.lowPower(false);
    imu.setSampleMode(
      (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
      ICM_20948_Sample_Mode_Continuous
    );

    // Sample rate = 1125 Hz / (1 + divisor); divisor 4 → ~225 Hz
    ICM_20948_smplrt_t rate;
    rate.a = 4; rate.g = 4;
    imu.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), rate);

    ICM_20948_fss_t fss;
    fss.a = gpm16;     // ±16 g — handles launch spike
    fss.g = dps2000;   // ±2000 dps — handles spin recovery
    imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);

    // Highest DLPF bandwidth — minimise phase lag into the EKF
    ICM_20948_dlpcfg_t dlp;
    dlp.a = acc_d473bw_n499bw;
    dlp.g = gyr_d361bw4_n376bw5;
    imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlp);
    imu.enableDLPF(ICM_20948_Internal_Acc, true);
    imu.enableDLPF(ICM_20948_Internal_Gyr, true);

    // INT1: active-high, push-pull, pulse, fires on data-ready
    imu.cfgIntActiveLow(false);
    imu.cfgIntOpenDrain(false);
    imu.cfgIntLatch(false);
    imu.intEnableRawDataReady(true);
    imu.sleep(false);

    // AK09916 mag runs via ICM's internal I2C master; max 100 Hz and NOT
    // synchronous with accel/gyro. Timestamp mag separately in the EKF.
    imu.startupMagnetometer();
    Serial.println("[IMU ] OK — ±16g, ±2000dps, AK09916 mag started");
  }

  baro_ok = baro.begin_SPI(PIN_CS_BARO);
  if (!baro_ok) {
    Serial.println("[BARO] FAIL — verify CS(9); shares SPI0 with IMU");
  } else {
    baro.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    baro.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    baro.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    // ODR 25 Hz (40 ms period). BMP390 datasheet §3.9.2: t_meas with 8× press +
    // 2× temp = 234 + 8×2020 + 2×2020 = ~20.4 ms, which exceeds the 20 ms period
    // needed for 50 Hz. 25 Hz gives a 40 ms window — measurement completes with
    // ~20 ms to spare. NOTE: performReading() uses forced-mode so setOutputDataRate
    // is advisory only; the actual poll interval is controlled in loop() below.
    baro.setOutputDataRate(BMP3_ODR_25_HZ);
    Serial.println("[BARO] OK — 25 Hz, pressure ×8, IIR coeff 3");
  }

  // I2C0 — OLED
  Wire.begin();
  Wire.setClock(400000);
  oled_ok = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oled_ok) {
    // Try 0x3D if this fires — some modules use the alternate address
    Serial.println("[OLED] Init failed — non-fatal, continuing");
  } else {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0); oled.print("Flight computer");
    oled.setCursor(0, 9); oled.print("Initializing...");
    oled.display();
    Serial.println("[OLED] OK");
  }

  // SDIO — built-in microSD
  sd_ok = SD.begin(BUILTIN_SDCARD);
  if (!sd_ok) {
    Serial.println("[SD  ] FAIL — check card seated; logging disabled");
  } else {
    Serial.println("[SD  ] OK");
  }

  // UART1 — GPS (NEO-M9N, SparkFun u-blox GNSS v2)
  // The module emits a startup banner before accepting UBX traffic — 500 ms
  // settle prevents the handshake racing against it.
  // Try 115200 first (previously saved config), fall back to factory default 38400.
  // Do NOT attempt a baud-rate switch here; stay on whichever rate responds.
  Serial1.begin(115200);
  delay(500);
  gps_ok = gnss.begin(Serial1);
  if (!gps_ok) {
    Serial.println("[GPS ] 115200 no ACK — trying factory default 38400...");
    Serial1.end();
    Serial1.begin(38400);
    delay(500);
    gps_ok = gnss.begin(Serial1);
  }
  if (gps_ok) {
    gnss.setUART1Output(COM_TYPE_UBX);   // switch GPS output to UBX binary only
    gnss.setNavigationFrequency(5);      // 5 Hz — reliable on both baud rates
    gnss.setAutoPVTcallback(pvt_callback);
    gnss.saveConfiguration();
    Serial.println("[GPS ] OK — 5 Hz NAV-PVT, callback armed");
  } else {
    Serial.println("[GPS ] FAIL — check TX(1)/RX(0) wiring");
  }

  // Interrupts — attach last, after all peripheral init complete
  //pinMode(PIN_INT_IMU,  INPUT);
  //attachInterrupt(digitalPinToInterrupt(PIN_INT_IMU),  imu_isr, RISING);

  pinMode(PIN_INT_BARO, INPUT);   // INT_BARO: not yet used (polling mode); reserve pin

  // GPS PPS: rising edge latched by pps_isr() for EKF time sync.
  // NEO-M9N outputs PPS once per second after achieving a time fix.
  pinMode(PIN_GPS_PPS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), pps_isr, RISING);

  Serial.println("[BOOT] All subsystems ready\n");
}

// =============================================================================
// Loop
// =============================================================================
void loop() {
  static uint32_t last_baro_ms     = 0;
  static uint32_t last_print_ms    = 0;
  static uint32_t last_oled_ms     = 0;
  static uint32_t last_gps_seek_ms = 0;
  static float    baro_alt_m       = 0.0f;
  static float    baro_temp_c      = 0.0f;
  static bool     gps_had_fix      = false;

  // IMU: service data-ready at ~225 Hz
  // getAGMT() bursts accel + gyro + mag + temp in one SPI transaction.
  // accX/Y/Z → mg;  gyrX/Y/Z → dps
  static uint32_t last_imu_ms = 0;
  if (imu_ok && millis() - last_imu_ms >= 5) {   // 200 Hz poll — matches your ~225 Hz ODR
    last_imu_ms = millis();
    imu.getAGMT();
  }

  // Baro: poll at 25 Hz (40 ms). performReading() blocks ~20 ms in forced-mode;
  // keeping the window at 40 ms prevents it from starving the IMU handler.
  // For flight firmware replace with INT_BARO interrupt + non-blocking normal mode.
  if (baro_ok && millis() - last_baro_ms >= 40) {
    last_baro_ms = millis();
    if (baro.performReading()) {
      baro_alt_m  = baro.readAltitude(1013.25f);
      baro_temp_c = (float)baro.temperature;
    }
  }

  // GPS: process incoming bytes and fire pvt_callback when a full packet arrives
  if (gps_ok) {
    gnss.checkUblox();
    gnss.checkCallbacks();
  }

  // GPS seek monitor — every 30 s, log status to Serial until first 3-D fix
  if (gps_ok && !gps_had_fix) {
    if (gps.fix >= 3) {
      gps_had_fix = true;
      Serial.printf("[GPS ] First 3D fix! lat:%.6f lon:%.6f alt:%.1fm sats:%d (%.0fs elapsed)\n",
                    gps.lat, gps.lon, gps.alt_m, gps.sats, millis() / 1000.0f);
    } else if (millis() - last_gps_seek_ms >= 30000) {
      last_gps_seek_ms = millis();
      Serial.printf("[GPS ] Seeking... %ds elapsed  sats visible:%d  fix type:%d\n",
                    (int)(millis() / 1000), gps.sats, gps.fix);
    }
  }

  // Serial monitor at 2 Hz
  if (millis() - last_print_ms >= 500) {
    last_print_ms = millis();

    if (imu_ok) {
      Serial.printf("IMU  ax:%8.1f ay:%8.1f az:%8.1f mg   gx:%7.2f gy:%7.2f gz:%7.2f dps\n",
                    imu.accX(), imu.accY(), imu.accZ(),
                    imu.gyrX(), imu.gyrY(), imu.gyrZ());
    } else {
      Serial.println("IMU  OFFLINE");
    }

    if (baro_ok) {
      Serial.printf("BARO alt:%7.1f m   temp:%.1f C\n", baro_alt_m, baro_temp_c);
    } else {
      Serial.println("BARO OFFLINE");
    }

    if (!gps_ok) {
      Serial.println("GPS  OFFLINE");
    } else if (gps.fresh) {
      Serial.printf("GPS  fix:%d sats:%2d  lat:%11.6f  lon:%11.6f  alt:%7.1fm  spd:%.2fm/s\n",
                    gps.fix, gps.sats, gps.lat, gps.lon, gps.alt_m, gps.spd_ms);
      gps.fresh = false;
    } else {
      Serial.println("GPS  awaiting first fix...");
    }

    if (imu_ok) {
      Serial.printf("MAG  mx:%7.2f my:%7.2f mz:%7.2f uT\n",
                    imu.magX(), imu.magY(), imu.magZ());
    }

    // PPS: age since last pulse (0 before first fix)
    uint32_t pps_us = pps_timestamp_us;   // snapshot volatile
    if (pps_us > 0)
      Serial.printf("PPS  last pulse %lu ms ago\n", (millis() * 1000UL - pps_us) / 1000UL);
    else
      Serial.println("PPS  awaiting first pulse...");

    Serial.println();
  }

  // OLED at 2 Hz — 128×32, textSize 1 = 6×8px per char, 4 rows at y=0/8/16/24
  //
  // Row 0: I:OK B:-- G:-- S:--   (component status)
  // Row 1: IMU accel in g        (or "IMU: OFFLINE")
  // Row 2: baro alt + temp       (or "BARO: OFFLINE")
  // Row 3: GPS status            (or "GPS: OFFLINE")
  if (oled_ok && millis() - last_oled_ms >= 500) {
    last_oled_ms = millis();
    char line[22];

    oled.clearDisplay();

    // Row 0: component status
    snprintf(line, sizeof(line), "I:%s B:%s G:%s S:%s",
             imu_ok  ? "OK" : "--",
             baro_ok ? "OK" : "--",
             gps_ok  ? "OK" : "--",
             sd_ok   ? "OK" : "--");
    oled.setCursor(0, 0); oled.print(line);

    // Row 1: IMU accelerometer (converted from mg to g) or offline notice
    if (imu_ok) {
      snprintf(line, sizeof(line), "%.1f %.1f %.1fg",
               imu.accX() / 1000.0f, imu.accY() / 1000.0f, imu.accZ() / 1000.0f);
    } else {
      snprintf(line, sizeof(line), "IMU: OFFLINE");
    }
    oled.setCursor(0, 8); oled.print(line);

    // Row 2: barometer altitude and temperature or offline notice
    if (baro_ok) {
      snprintf(line, sizeof(line), "%.0fm %.1fC", baro_alt_m, baro_temp_c);
    } else {
      snprintf(line, sizeof(line), "BARO: OFFLINE");
    }
    oled.setCursor(0, 16); oled.print(line);

    // Row 3: GPS fix status, seeking info, or offline notice
    if (!gps_ok) {
      snprintf(line, sizeof(line), "GPS: OFFLINE");
    } else if (gps.fix >= 2) {
      snprintf(line, sizeof(line), "GPS %s %dsv",
               gps.fix >= 3 ? "3D" : "2D", gps.sats);
    } else {
      uint32_t elapsed_s = millis() / 1000;
      snprintf(line, sizeof(line), "SEEK %dsv %dm%02ds",
               gps.sats, elapsed_s / 60, elapsed_s % 60);
    }
    oled.setCursor(0, 24); oled.print(line);

    oled.display();
  }

  // LED stage indicator (non-blocking)
  // solid ON = 3D fix | 2 Hz = 2D fix | 1 Hz = no fix or GPS offline
  {
    static uint32_t last_led_ms = 0;
    static bool     led_state   = false;
    uint32_t interval = (gps_ok && gps.fix >= 3) ? 0 :    // solid ON — 3D fix
                        (gps_ok && gps.fix == 2) ? 250 :   // 2 Hz    — 2D fix
                                                    500;    // 1 Hz    — no fix / GPS offline
    if (interval == 0) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (millis() - last_led_ms >= interval) {
      last_led_ms = millis();
      led_state = !led_state;
      digitalWrite(LED_BUILTIN, led_state);
    }
  }
}
