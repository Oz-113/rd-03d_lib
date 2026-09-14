/*
 * RD-03D Multi-Target tracker example (ESP32).
 *
 * Tracks up to three humans simultaneously and prints every valid target.
 *
 * Wiring: see Basic_SingleTarget example.
 */

#include <RD03D.h>

#define RD03D_RX 16
#define RD03D_TX 17

RD03D radar(Serial1);

void setup() {
  Serial.begin(115200);
  Serial1.begin(256000, SERIAL_8N1, RD03D_RX, RD03D_TX);
  radar.begin(RD03D::MULTI_TARGET);
  Serial.println("RD-03D multi-target tracker");
}

void loop() {
  if (radar.read()) {
    uint8_t n = radar.targetCount();
    Serial.print(n);
    Serial.println(" target(s)");

    for (uint8_t i = 0; i < RD03D::MAX_TARGETS; i++) {
      const RD03D::Target &t = radar.target(i);
      Serial.print("  #");
      Serial.print(i + 1);
      if (t.valid) {
        Serial.print("  d=");
        Serial.print(t.distanceCm(), 0);
        Serial.print("cm  a=");
        Serial.print(t.angleDeg(), 0);
        Serial.print("deg  (x=");
        Serial.print(t.x);
        Serial.print(",y=");
        Serial.print(t.y);
        Serial.print(")mm  v=");
        Serial.print(t.speed);
        Serial.print("cm/s  res=");
        Serial.println(t.distanceRes);
      } else {
        Serial.println("  --");
      }
    }
    Serial.println();
  }
}
