# RD-03D serial protocol reference

This document describes the UART protocol of the Ai-Thinker **RD-03D**
24 GHz multi-target radar. It is the byte-level reference behind the `RD03D`
library.

## Serial parameters

| Parameter | Value          |
| --------- | -------------- |
| Baud rate | **256000 bps** |
| Data bits | 8              |
| Parity    | None           |
| Stop bits | 1              |
| Logic     | 3.3 V TTL      |

The default baud rate is fixed at 256000. Some firmware revisions may also
accept 115200, but 256000 is the documented default.

---

## Report frame (radar → host)

The radar continuously streams report frames while it is running. Each frame
is exactly **30 bytes**:

```
 AA FF 03 00 | T1 (8 bytes) | T2 (8 bytes) | T3 (8 bytes) | 55 CC
```

| Offset | Size | Field                              |
| ------ | ---- | ---------------------------------- |
| 0 – 3  | 4    | Frame header `AA FF 03 00`         |
| 4 – 11 | 8    | Target 1                           |
| 12 – 19| 8    | Target 2                           |
| 20 – 27| 8    | Target 3                           |
| 28 – 29| 2    | Frame tail `55 CC`                 |

### Target slot (8 bytes)

| Offset | Size | Field | Notes                                 |
| ------ | ---- | ----- | ------------------------------------- |
| +0 – +1| 2    | X     | signed, little-endian, **mm**         |
| +2 – +3| 2    | Y     | signed, little-endian, **mm**         |
| +4 – +5| 2    | speed | signed, little-endian, **cm/s**       |
| +6 – +7| 2    | distance | unsigned, little-endian, **mm**   |

The **distance** field is `0` when a slot carries no target. An empty slot is
typically all zero bytes.

### Signed integer encoding (X / Y / speed)

X, Y and speed are encoded as **sign-magnitude**: bit 15 is the sign flag and
bits 14–0 are the magnitude.

```
raw   = (uint16_t)lo | ((uint16_t)hi << 8);
mag   = raw & 0x7FFF;
value = (raw & 0x8000) ? -mag : mag;
```

Examples:

| Raw bytes | Raw hex | Decoded |
| --------- | ------- | ------- |
| `00 00`   | 0x0000  | 0       |
| `64 00`   | 0x0064  | +100    |
| `E8 03`   | 0x03E8  | +1000   |
| `E8 83`   | 0x83E8  | −1000   |

### Derived values

```
distance_mm  = sqrt(x^2 + y^2)
distance_cm  = distance_mm / 10
angle_deg    = atan2(x, -y) * 180 / pi   // 0 deg = straight ahead
```

> The RD-03D reports "in front of the sensor" as **negative Y**, hence the
> `−y` in the angle calculation. The sign of X determines left/right and
> depends on the physical orientation of the module.

---

## Command frame (host → radar)

Commands use a different envelope. The host sends:

```
 FD FC FB FA | length (2 bytes, LE) | payload (length bytes) | 04 03 02 01
```

- `FD FC FB FA` — command header
- `length` — payload length, little-endian `uint16`
- `payload` — command data
- `04 03 02 01` — command trailer

### Built-in commands

| Command              | Payload         | Notes                                   |
| -------------------- | --------------- | --------------------------------------- |
| Single target mode   | `80 00`         | report one target                       |
| Multi target mode    | `90 00`         | report up to three targets              |
| Open command mode    | `FF 00 01 00`   | enter configuration mode                |
| Close command mode   | `FE 00`         | exit configuration mode                 |

Full byte sequences:

```
Single target : FD FC FB FA 02 00 80 00 04 03 02 01
Multi target  : FD FC FB FA 02 00 90 00 04 03 02 01
Open config   : FD FC FB FA 04 00 FF 00 01 00 04 03 02 01
Close config  : FD FC FB FA 02 00 FE 00 04 03 02 01
```

> Some firmware versions require the **open → command → close** sequence before
> the mode change takes effect. The library's `setMode()` sends the mode command
> directly (which works on most modules); `enterConfigMode()` / `exitConfigMode()`
> are provided for the full handshake.

### ACK frame

After a command, the radar replies with an ACK in the same `FD FC FB FA`
envelope. The payload begins with a 2-byte command reply and a 2-byte status:

```
 FD FC FB FA | length | commandReply(2) | status(2) | ... | 04 03 02 01
```

A status of `0` indicates success. The library's `waitForAck()` locates the
trailer and returns whether the frame was structurally valid.

---

## Notes / caveats

- The exact byte order and sign encoding above were verified against the
  Ai-Thinker RD-03 protocol and multiple independent community
  implementations. If your module behaves differently, dump raw frames with the
  `RawFrame_Dump` example before adapting the parser.
- The `distance` field (bytes +6..+7 of each target) is vendor-specific; the
  library treats it as an extra distance reading and as the empty-slot marker.
- X/Y/speed are in mm and cm/s respectively; distance/angle are derived values.
