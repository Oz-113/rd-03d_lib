/*
 * RD03D.h - Arduino library for the Ai-Thinker RD-03D 24 GHz mmWave
 * multi-human tracking radar.
 *
 * The RD-03D reports up to three tracked targets over UART at 256000 baud
 * (8N1). This library parses the 30-byte report frames and exposes each
 * target's X/Y position (mm), speed (cm/s) and the vendor "distance
 * resolution" field, together with derived distance and angle values.
 *
 * MIT License - see LICENSE file.
 */

#ifndef RD03D_H
#define RD03D_H

#include <Arduino.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class RD03D {
public:
  /* Maximum number of targets the RD-03D can report. */
  static const uint8_t MAX_TARGETS = 3;

  /* Operating mode of the radar. */
  enum Mode : uint8_t {
    SINGLE_TARGET = 0, /* track one target only            */
    MULTI_TARGET  = 1  /* track up to three targets        */
  };

  /* One detected target (a "slot" inside the report frame). */
  struct Target {
    int16_t  x;           /* X coordinate, millimetres                    */
    int16_t  y;           /* Y coordinate, millimetres (negative = ahead) */
    int16_t  speed;       /* speed, cm/s                                  */
    uint16_t distanceRes; /* vendor distance field, mm (0 = empty slot)   */
    uint32_t timestamp;   /* millis() at decode time                      */
    bool     valid;       /* true while this slot carries a target        */

    Target() : x(0), y(0), speed(0), distanceRes(0), timestamp(0), valid(false) {}

    /* Straight-line distance computed from X/Y (mm). */
    float distanceMm() const { return sqrtf((float)x * (float)x + (float)y * (float)y); }
    float distanceCm() const { return distanceMm() / 10.0f; }
    float distanceM()  const { return distanceMm() / 1000.0f; }

    /*
     * Azimuth angle in radians / degrees.
     * 0 deg = straight ahead (boresight); sign follows the X axis
     * (positive X -> positive angle). atan2(x, -y) is used because the
     * RD-03D reports forward motion as negative Y.
     */
    float angleRad() const { return atan2f((float)x, -(float)y); }
    float angleDeg() const { return angleRad() * 180.0f / (float)M_PI; }
  };

  /*
   * Constructor. Pass a Stream that is already configured for the radar
   * (HardwareSerial on ESP32/AVR, SoftwareSerial, ...). The library only
   * reads/writes the stream; call serial.begin(...) yourself, e.g.:
   *
   *   Serial1.begin(256000, SERIAL_8N1, RX_PIN, TX_PIN);   // ESP32
   */
  explicit RD03D(Stream &stream);

  /*
   * Configure the radar and select the operating mode. This sends the
   * single/multi target command and drains the serial input. Call once
   * from setup() after the underlying stream has been started.
   */
  bool begin(Mode mode = MULTI_TARGET);

  /* Switch operating mode (single/multi target). */
  bool setMode(Mode mode);
  bool setSingleTargetMode();
  bool setMultiTargetMode();

  /* Enter / leave the RD-03 "command mode" (used by some firmware versions). */
  bool enterConfigMode();
  bool exitConfigMode();

  /* Send a raw command payload wrapped in the FD FC FB FA envelope. */
  bool sendCommand(const uint8_t *payload, uint16_t length);

  /* Write raw bytes to the radar serial port. */
  bool sendRaw(const uint8_t *buffer, size_t length);

  /*
   * Pump the parser. Call frequently from loop(). Reads every pending byte
   * and decodes complete report frames. Returns true when at least one
   * valid frame was decoded during this call.
   */
  bool read();

  /* True when read() decoded a frame in the most recent call. */
  bool available() const { return _newFrame; }

  /* Number of valid targets in the last decoded frame. */
  uint8_t targetCount() const { return _targetCount; }

  /* Access a decoded target by index (0..2). Out-of-range indexes are clamped. */
  const Target &target(uint8_t index) const;

  /* Raw pointer to the three target slots (index 0..2). */
  const Target *targets() const { return _targets; }

  /* Current operating mode. */
  Mode mode() const { return _mode; }

  /* Diagnostics. */
  uint32_t framesReceived() const { return _framesOk; }
  uint32_t framesBad()      const { return _framesBad; }

  /*
   * Block until a command ACK frame (FD FC FB FA ... 04 03 02 01) is
   * received, or until timeoutMs elapses. Returns true when a valid ACK
   * trailer was seen. Useful after sendCommand().
   */
  bool waitForAck(uint32_t timeoutMs = 300);

  /* Discard all bytes currently buffered on the radar serial port. */
  void flushInput();

private:
  static const uint8_t FRAME_SIZE  = 30; /* AA FF 03 00 + 3*8 + 55 CC */
  static const uint8_t HEADER_SIZE = 4;
  static const uint8_t TARGET_SIZE = 8;

  static const uint8_t HEADER[4];     /* AA FF 03 00 */
  static const uint8_t TAIL[2];       /* 55 CC       */
  static const uint8_t CMD_HEADER[4]; /* FD FC FB FA */
  static const uint8_t CMD_TRAILER[4];/* 04 03 02 01 */

  static const uint8_t CMD_SINGLE[2]; /* 80 00 */
  static const uint8_t CMD_MULTI[2];  /* 90 00 */
  static const uint8_t CMD_OPEN[4];   /* FF 00 01 00 */
  static const uint8_t CMD_CLOSE[2];  /* FE 00 */

  enum ParseState : uint8_t { WAIT_HEADER, READ_DATA };

  Stream    &_stream;
  Mode       _mode;

  uint8_t    _buf[FRAME_SIZE];
  uint8_t    _bufPos;
  uint8_t    _headerIdx;
  ParseState _state;

  Target     _targets[MAX_TARGETS];
  uint8_t    _targetCount;

  uint32_t   _framesOk;
  uint32_t   _framesBad;
  bool       _newFrame;

  /* Decode the RD-03D sign-magnitude signed integer (bit 15 = sign). */
  static int16_t decodeSigned(uint8_t lo, uint8_t hi);

  bool feedByte(uint8_t b);
  void parseFrame();
};

#endif
