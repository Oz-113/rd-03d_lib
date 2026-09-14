/*
 * RD-03D Raw Frame Dump (ESP32).
 *
 * This example does NOT use the RD03D library parser. It reads the radar
 * serial port directly and dumps each 30-byte report frame as hex, which is
 * useful for debugging wiring issues or verifying the protocol.
 *
 * Frame layout:
 *   [0..3]   AA FF 03 00    header
 *   [4..11]  target 1: x(2) y(2) speed(2) distance(2)
 *   [12..19] target 2
 *   [20..27] target 3
 *   [28..29] 55 CC          tail
 */

#include <Arduino.h>

#define RD03D_RX 16
#define RD03D_TX 17

static const uint8_t HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t TAIL[2]   = {0x55, 0xCC};

static uint8_t buf[30];
static uint8_t pos = 0;
static uint8_t hdr = 0;
static bool    inFrame = false;

static int16_t decodeSigned(uint8_t lo, uint8_t hi) {
  uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
  int16_t  mag = (int16_t)(raw & 0x7FFF);
  return (raw & 0x8000) ? (int16_t)(-mag) : mag;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(256000, SERIAL_8N1, RD03D_RX, RD03D_TX);
  Serial.println("RD-03D raw frame dump");
}

void loop() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    if (!inFrame) {
      if (b == HEADER[hdr]) {
        buf[pos++] = b;
        if (++hdr >= 4) { inFrame = true; hdr = 0; }
      } else {
        pos = 0;
        hdr = 0;
        if (b == HEADER[0]) { buf[pos++] = b; hdr = 1; }
      }
      continue;
    }

    buf[pos++] = b;
    if (pos >= 30) {
      inFrame = false;
      pos = 0;

      if (buf[28] != TAIL[0] || buf[29] != TAIL[1]) {
        Serial.println("(bad tail)");
        continue;
      }

      for (int i = 0; i < 30; i++) {
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      for (int t = 0; t < 3; t++) {
        int off = 4 + t * 8;
        int16_t x = decodeSigned(buf[off], buf[off + 1]);
        int16_t y = decodeSigned(buf[off + 2], buf[off + 3]);
        int16_t s = decodeSigned(buf[off + 4], buf[off + 5]);
        uint16_t d = (uint16_t)buf[off + 6] | ((uint16_t)buf[off + 7] << 8);
        Serial.print("  T");
        Serial.print(t + 1);
        Serial.print(": x=");
        Serial.print(x);
        Serial.print(" y=");
        Serial.print(y);
        Serial.print(" v=");
        Serial.print(s);
        Serial.print(" d=");
        Serial.println(d);
      }
    }
  }
}
