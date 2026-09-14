# RD03D — Arduino library for the Ai-Thinker RD-03D mmWave radar

I just wrote a prompt, this library is ai generated.

An easy-to-use Arduino/ESP32 library for the **Ai-Thinker RD-03D** 24 GHz
millimetre-wave **multi-human tracking** radar. The RD-03D tracks up to **three**
moving humans at once and reports, for each target, its **X/Y position**, **speed**
and a **distance** field over a UART serial link.

This library decodes the 30-byte report frames for you and exposes a clean API so
you can read positions, distances, speeds and azimuth angles with a few lines of
code.

---

## Table of contents

1. [Features](#features)
2. [Hardware overview](#hardware-overview)
3. [Pinout](#pinout)
4. [Wiring](#wiring)
5. [Installation](#installation)
6. [Quick start](#quick-start)
7. [API reference](#api-reference)
8. [Coordinate system & angle convention](#coordinate-system--angle-convention)
9. [Examples](#examples)
10. [Protocol summary](#protocol-summary)
11. [Troubleshooting](#troubleshooting)
12. [Portability notes](#portability-notes)
13. [License](#license)

---

## Features

- Decodes **single- and multi-target** report frames automatically.
- Up to **3 targets** per frame (`RD03D::MAX_TARGETS`).
- Per-target **X/Y (mm)**, **speed (cm/s)** and the vendor **distance/resolution** field.
- Derived **distance** (mm/cm/m) and **azimuth angle** (degrees/radians).
- Robust frame parser with header resynchronisation and tail verification.
- Non-blocking `read()` — safe to call from `loop()`.
- Command helpers for single/multi-target mode and the config command envelope.
- Works on **ESP32** (recommended) and, with care, on AVR/SoftwareSerial boards.

---

## Hardware overview

The RD-03D is a compact 24 GHz (K-band) mmWave radar module from Ai-Thinker.

| Parameter            | Value                                  |
| -------------------- | -------------------------------------- |
| Frequency            | 24.0 – 24.25 GHz                       |
| Supply voltage       | 5 V                                    |
| Supply current       | ≥ 200 mA                               |
| Interface            | UART                                   |
| UART baud rate       | **256000 bps** (8N1)                   |
| I/O logic level      | 3.3 V                                  |
| Size                 | 15.0 × 44.0 mm (module)                |
| Antenna              | On-board PCB antenna                   |
| Max targets          | 3                                      |
| Typical range        | up to ~8 m (human target)              |
| Working temperature  | -40 … +85 °C                           |

> **Note:** the module is powered from **5 V**, but its UART **TX/RX pins are
> 3.3 V logic** — safe to connect directly to a 3.3 V ESP32.

---

## Pinout

The bare RD-03D module exposes a 4-pin connector (`1*4P-1.25 mm`):

| RD-03D pin | Function                              |
| ---------- | ------------------------------------- |
| **5V / VCC** | Power (5 V)                         |
| **GND**    | Ground                                |
| **RX**     | UART receive (connect to MCU **TX**)  |
| **TX**     | UART transmit (connect to MCU **RX**) |

Some breakouts also expose `OT1`/`OT2` GPIO outputs; they are not required for
this library (everything is read over UART).

---

## Wiring

### ESP32 (recommended)

| RD-03D | ESP32             |
| ------ | ----------------- |
| 5V     | 5V or VIN         |
| GND    | GND               |
| TX     | GPIO16 (UART RX)  |
| RX     | GPIO17 (UART TX)  |

The example sketches use `Serial1` on GPIO16 (RX) and GPIO17 (TX). Change
`RD03D_RX` / `RD03D_TX` to match your board if needed.

### Other boards

The library takes a `Stream&`, so it also works with:

- **AVR Mega/Due/Teensy** — use a spare `Serial1`/`Serial2`/… hardware UART.
- **AVR Uno/Nano** — use `SoftwareSerial`. ⚠️ `SoftwareSerial` is **not reliable at
  256000 baud**; a hardware UART is strongly recommended.

```cpp
#include <SoftwareSerial.h>
SoftwareSerial radarSerial(10, 11); // RX, TX
RD03D radar(radarSerial);

void setup() {
  Serial.begin(115200);
  radarSerial.begin(256000);
  radar.begin(RD03D::MULTI_TARGET);
}
```

## Installation

### Arduino IDE

1. Copy this folder (`mmWavelib`, or rename it to `RD03D`) into your Arduino
   `libraries/` folder.
2. Restart the Arduino IDE.
3. Use **Sketch → Include Library → RD03D**.

Or install as a ZIP: **Sketch → Include Library → Add .ZIP Library…**.

### PlatformIO

```ini
lib_deps =
    symlink://.   ; or publish and use "RD03D"
```

(For a local copy, place the library under `lib/RD03D`.)

---

## Quick start

```cpp
#include <RD03D.h>

#define RD03D_RX 16
#define RD03D_TX 17

RD03D radar(Serial1);

void setup() {
  Serial.begin(115200);
  Serial1.begin(256000, SERIAL_8N1, RD03D_RX, RD03D_TX);
  radar.begin(RD03D::MULTI_TARGET);   // or SINGLE_TARGET
}

void loop() {
  radar.read();                       // pump the parser

  const RD03D::Target &t = radar.target(0);
  if (t.valid) {
    Serial.print("Distance: "); Serial.print(t.distanceCm());
    Serial.print(" cm, Angle: "); Serial.print(t.angleDeg());
    Serial.print(" deg, Speed: "); Serial.print(t.speed);
    Serial.println(" cm/s");
  }
}
```

---

## API reference

### Constructor

```cpp
RD03D radar(Stream &stream);
```

Pass the serial port the radar is connected to. You are responsible for calling
`stream.begin(...)` yourself (e.g. `Serial1.begin(256000, SERIAL_8N1, RX, TX)`
on ESP32).

### Configuration

```cpp
bool begin(Mode mode = MULTI_TARGET);
```

Sends the single/multi-target command and drains the input buffer. Call once in
`setup()`.

```cpp
bool setMode(Mode mode);
bool setSingleTargetMode();
bool setMultiTargetMode();
```

Switch the operating mode at any time. `Mode` is `SINGLE_TARGET` or
`MULTI_TARGET`.

```cpp
bool enterConfigMode();   // send "open command mode"
bool exitConfigMode();    // send "close command mode"
bool sendCommand(const uint8_t *payload, uint16_t length);
bool sendRaw(const uint8_t *buffer, size_t length);
bool waitForAck(uint32_t timeoutMs = 300);
```

Low-level command helpers for advanced use. `sendCommand()` wraps `payload` in
the `FD FC FB FA | len(LE) | payload | 04 03 02 01` envelope. `waitForAck()`
blocks until an ACK frame arrives (or times out).

### Runtime

```cpp
bool read();            // parse all pending bytes; true if a frame was decoded
bool available() const; // true when read() decoded a frame in its last call
```

Call `read()` frequently (every `loop()` iteration).

### Accessing targets

```cpp
uint8_t targetCount() const;              // number of valid targets in last frame
const Target &target(uint8_t index) const;// target 0..2 (clamped)
const Target *targets() const;            // pointer to the 3-slot array
Mode mode() const;                        // current operating mode
```

### The `Target` struct

```cpp
struct Target {
  int16_t  x;           // X coordinate, mm
  int16_t  y;           // Y coordinate, mm (negative = ahead)
  int16_t  speed;       // speed, cm/s
  uint16_t distanceRes; // vendor distance field, mm (0 = empty slot)
  uint32_t timestamp;   // millis() at decode time
  bool     valid;       // true while this slot carries a target

  float distanceMm() const;  // sqrt(x^2 + y^2)
  float distanceCm() const;
  float distanceM()  const;
  float angleRad()   const;  // atan2(x, -y)
  float angleDeg()   const;  // angle in degrees
};
```

### Diagnostics

```cpp
uint32_t framesReceived() const; // good frames decoded
uint32_t framesBad()      const; // frames that failed tail verification
```

---

## Coordinate system & angle convention

- **X** and **Y** are reported in **millimetres**.
- **Speed** is reported in **cm/s**.
- `distanceMm()`/`distanceCm()` are the straight-line distance computed from X/Y.
- `angleDeg()` returns the azimuth angle where **0° = straight ahead**
  (boresight), with positive angles on the **+X side** and negative on the
  **−X side**.

The RD-03D reports "in front of the sensor" as **negative Y**, so the library
computes `angle = atan2(x, -y)`.

> ⚠️ The exact left/right sense depends on how you mount the sensor. If your
> targets appear mirrored, either rotate the module 180° or negate `t.x` /
> flip the sign of `angleDeg()` in your own code. The raw `t.x` / `t.y` values
> are always available so you can implement any convention you prefer.

## Examples

| Example                                   | What it shows                                        |
| ----------------------------------------- | ---------------------------------------------------- |
| `Basic_SingleTarget`                      | Print distance/angle/speed of the closest target     |
| `MultiTarget_Tracker`                     | Print all (up to 3) tracked targets                  |
| `Angle_Relay`                             | Toggle a relay when a target enters an azimuth zone  |
| `Distance_Relay`                          | Toggle a relay when a target is within a distance    |
| `RawFrame_Dump`                           | Dump raw 30-byte frames as hex (debugging)           |

---

## Protocol summary

| Direction     | Frame                                                         |
| ------------- | ------------------------------------------------------------- |
| Radar → host  | `AA FF 03 00` + 3×8-byte targets + `55 CC` (30 bytes)         |
| Host → radar  | `FD FC FB FA` + length(LE) + payload + `04 03 02 01`          |

Each 8-byte target slot:

| Bytes | Field         | Encoding                                   |
| ----- | ------------- | ------------------------------------------ |
| 0–1   | X (mm)        | signed, sign-magnitude (bit 15 = sign)     |
| 2–3   | Y (mm)        | signed, sign-magnitude                     |
| 4–5   | speed (cm/s)  | signed, sign-magnitude                     |
| 6–7   | distance (mm) | unsigned; **0 = no target in this slot**   |

Key commands (payload inside the command envelope):

| Command            | Payload          |
| ------------------ | ---------------- |
| Single target mode | `80 00`          |
| Multi target mode  | `90 00`          |
| Open command mode  | `FF 00 01 00`    |
| Close command mode | `FE 00`          |

See [`docs/PROTOCOL.md`](docs/PROTOCOL.md) for the complete byte-level reference.

---

## Troubleshooting

- **No data at all** — check TX/RX are crossed (radar TX → MCU RX), baud is
  exactly **256000**, and the module is powered from **5 V** with ≥ 200 mA.
- **Garbage values / 32 m distances** — the parser is receiving partial frames.
  Try `RawFrame_Dump` to inspect the raw bytes and verify wiring/grounding.
- **Wrong angle sign** — see the coordinate-system note above; the left/right
  sense depends on mounting.
- **ESP32 drops bytes** — the ESP32 default UART RX buffer may be small. Increase
  it before `begin()` if needed:
  ```cpp
  Serial1.setRxBufferSize(256);   // ESP32 only
  ```
- **AVR Uno/Nano unreliable** — `SoftwareSerial` cannot sustain 256000 baud
  reliably; use a hardware UART.

---

## Portability notes

- The library only uses `Stream` (`available/read/write`), so it compiles on
  ESP32, ESP8266, AVR, SAMD, RP2040, etc.
- On ESP32 the radar is normally attached to `Serial1`/`Serial2` with custom pins.
- On AVR you need a hardware UART (Mega `Serial1`…) or `SoftwareSerial` (slow).

---

## License

MIT — see [LICENSE](LICENSE).


