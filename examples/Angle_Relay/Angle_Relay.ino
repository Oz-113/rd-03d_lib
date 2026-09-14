/*
 * RD-03D angle-zone relay example (ESP32).
 *
 * Turns a relay (or LED) on when a target enters a chosen azimuth zone and
 * off after the target leaves for a debounce period.
 *
 *  - ON  when |angle| <= ZONE_DEG   (target roughly straight ahead)
 *  - OFF after the target stays out of the zone for OFF_DELAY_MS
 *
 * Wiring: see Basic_SingleTarget example. Relay: GPIO 13 (active HIGH).
 */

#include <RD03D.h>

#define RD03D_RX     16
#define RD03D_TX     17
#define RELAY_PIN    13

#define ZONE_DEG     30        // half-width of the "in front" zone
#define ON_DELAY_MS  250       // target must be in zone this long to switch ON
#define OFF_DELAY_MS 2000      // target must be out of zone this long to switch OFF

RD03D radar(Serial1);

bool  relayOn = false;
uint32_t inSince  = 0;
uint32_t outSince = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(256000, SERIAL_8N1, RD03D_RX, RD03D_TX);
  radar.begin(RD03D::MULTI_TARGET);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("RD-03D angle-zone relay");
}

void loop() {
  radar.read();

  const RD03D::Target &t = radar.target(0);
  uint32_t now = millis();

  bool inZone = t.valid && (fabs(t.angleDeg()) <= ZONE_DEG);

  if (inZone) {
    outSince = 0;
    if (inSince == 0) inSince = now;
    if (!relayOn && (now - inSince >= ON_DELAY_MS)) {
      relayOn = true;
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Relay ON");
    }
  } else {
    inSince = 0;
    if (outSince == 0) outSince = now;
    if (relayOn && (now - outSince >= OFF_DELAY_MS)) {
      relayOn = false;
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Relay OFF");
    }
  }
}
