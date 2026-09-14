/*
 * RD-03D distance-threshold relay example (ESP32).
 *
 * Turns a relay (or LED) on when a target is closer than DISTANCE_CM and
 * off after the target moves away for a debounce period.
 *
 * Wiring: see Basic_SingleTarget example. Relay: GPIO 13 (active HIGH).
 */

#include <RD03D.h>

#define RD03D_RX     16
#define RD03D_TX     17
#define RELAY_PIN    13

#define DISTANCE_CM  150       // trigger distance
#define ON_DELAY_MS  250
#define OFF_DELAY_MS 2000

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
  Serial.println("RD-03D distance relay");
}

void loop() {
  radar.read();

  const RD03D::Target &t = radar.target(0);
  uint32_t now = millis();

  bool inRange = t.valid && (t.distanceCm() <= DISTANCE_CM);

  if (inRange) {
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
