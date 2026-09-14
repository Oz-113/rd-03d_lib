/*
 * RD-03D Basic Single-Target example (ESP32).
 *
 * Prints the closest (single) tracked target's distance, azimuth angle and
 * speed to the serial monitor once per second.
 *
 * Wiring (ESP32):
 *   RD-03D 5V  ->  ESP32 5V (or VIN)
 *   RD-03D GND ->  ESP32 GND
 *   RD-03D TX  ->  ESP32 GPIO16 (RX)
 *   RD-03D RX  ->  ESP32 GPIO17 (TX)
 *
 * Adjust RD03D_RX / RD03D_TX to match your board.
 */

#include <RD03D.h>

#define RD03D_RX 16   // ESP32 pin wired to the radar TX line
#define RD03D_TX 17   // ESP32 pin wired to the radar RX line

RD03D radar(Serial1);

void setup() {
  Serial.begin(115200);

  // Radar UART: 256000 baud, 8 data bits, no parity, 1 stop bit.
  Serial1.begin(256000, SERIAL_8N1, RD03D_RX, RD03D_TX);

  // Configure the radar for single-target detection.
  radar.begin(RD03D::SINGLE_TARGET);

  Serial.println("RD-03D single-target example");
}

void loop() {
  // Always pump the parser.
  radar.read();

  // Print a little less often than the frame rate.
  static uint32_t nextPrint = 0;
  if (millis() < nextPrint) return;
  nextPrint = millis() + 500;

  const RD03D::Target &t = radar.target(0);
  if (t.valid) {
    Serial.print("Distance: ");
    Serial.print(t.distanceCm(), 1);
    Serial.print(" cm | Angle: ");
    Serial.print(t.angleDeg(), 1);
    Serial.print(" deg | X: ");
    Serial.print(t.x);
    Serial.print(" mm | Y: ");
    Serial.print(t.y);
    Serial.print(" mm | Speed: ");
    Serial.print(t.speed);
    Serial.println(" cm/s");
  } else {
    Serial.println("No target");
  }
}
