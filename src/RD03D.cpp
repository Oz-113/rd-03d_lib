/*
 * RD03D.cpp - implementation for the Ai-Thinker RD-03D radar library.
 * See RD03D.h for the public API and protocol notes.
 */

#include "RD03D.h"

/* Report frame: header AA FF 03 00, then three 8-byte targets, then 55 CC. */
const uint8_t RD03D::HEADER[4]      = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t RD03D::TAIL[2]        = {0x55, 0xCC};

/* Command envelope: FD FC FB FA | len(LE) | payload | 04 03 02 01. */
const uint8_t RD03D::CMD_HEADER[4]  = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t RD03D::CMD_TRAILER[4] = {0x04, 0x03, 0x02, 0x01};

const uint8_t RD03D::CMD_SINGLE[2]  = {0x80, 0x00}; /* single target detection */
const uint8_t RD03D::CMD_MULTI[2]   = {0x90, 0x00}; /* multi  target detection */
const uint8_t RD03D::CMD_OPEN[4]    = {0xFF, 0x00, 0x01, 0x00}; /* enter config mode */
const uint8_t RD03D::CMD_CLOSE[2]   = {0xFE, 0x00}; /* exit config mode */

RD03D::RD03D(Stream &stream)
  : _stream(stream),
    _mode(MULTI_TARGET),
    _bufPos(0),
    _headerIdx(0),
    _state(WAIT_HEADER),
    _targetCount(0),
    _framesOk(0),
    _framesBad(0),
    _newFrame(false)
{
  memset(_buf, 0, sizeof(_buf));
}

bool RD03D::begin(Mode mode) {
  _mode = mode;
  flushInput();
  bool ok = setMode(mode);
  flushInput();
  return ok;
}

bool RD03D::setMode(Mode mode) {
  _mode = mode;
  const uint8_t *payload = (mode == SINGLE_TARGET) ? CMD_SINGLE : CMD_MULTI;
  return sendCommand(payload, 2);
}

bool RD03D::setSingleTargetMode() { return setMode(SINGLE_TARGET); }
bool RD03D::setMultiTargetMode()  { return setMode(MULTI_TARGET); }

bool RD03D::enterConfigMode() { return sendCommand(CMD_OPEN, sizeof(CMD_OPEN)); }
bool RD03D::exitConfigMode()  { return sendCommand(CMD_CLOSE, sizeof(CMD_CLOSE)); }

bool RD03D::sendCommand(const uint8_t *payload, uint16_t length) {
  if (payload == NULL || length > 24) return false;

  /* 4-byte header + 2-byte length + payload + 4-byte trailer. */
  uint8_t frame[4 + 2 + 24 + 4];
  size_t  n = 0;

  memcpy(frame + n, CMD_HEADER, 4); n += 4;
  frame[n++] = (uint8_t)(length & 0xFF);
  frame[n++] = (uint8_t)((length >> 8) & 0xFF);
  memcpy(frame + n, payload, length); n += length;
  memcpy(frame + n, CMD_TRAILER, 4); n += 4;

  return sendRaw(frame, n);
}

bool RD03D::sendRaw(const uint8_t *buffer, size_t length) {
  if (buffer == NULL) return false;
  return _stream.write(buffer, length) == length;
}

bool RD03D::read() {
  _newFrame = false;
  while (_stream.available()) {
    uint8_t b = _stream.read();
    if (feedByte(b)) {
      _newFrame = true;
    }
  }
  return _newFrame;
}

const RD03D::Target &RD03D::target(uint8_t index) const {
  if (index >= MAX_TARGETS) index = MAX_TARGETS - 1;
  return _targets[index];
}

void RD03D::flushInput() {
  while (_stream.available()) {
    _stream.read();
  }
}

bool RD03D::waitForAck(uint32_t timeoutMs) {
  uint32_t start  = millis();
  int      hdrMatch = 0;
  bool     haveLen  = false;
  uint16_t dataLen  = 0;
  int      bodyIdx  = 0;
  uint8_t  body[32];
  uint8_t  lenBuf[2];

  while ((uint32_t)(millis() - start) < timeoutMs) {
    if (!_stream.available()) {
      continue;
    }

    uint8_t b = _stream.read();

    if (!haveLen) {
      if (hdrMatch < 4) {
        if (b == CMD_HEADER[hdrMatch]) {
          hdrMatch++;
        } else {
          hdrMatch = (b == CMD_HEADER[0]) ? 1 : 0;
        }
        if (hdrMatch == 4) {
          /* header matched, now read the 2-byte length */
        }
      } else {
        lenBuf[hdrMatch - 4] = b;
        hdrMatch++;
        if (hdrMatch == 6) {
          dataLen = (uint16_t)lenBuf[0] | ((uint16_t)lenBuf[1] << 8);
          if (dataLen < 2 || dataLen > 24) {
            /* unexpected length -> resync */
            hdrMatch = 0;
          } else {
            haveLen = true;
            bodyIdx = 0;
          }
        }
      }
    } else {
      body[bodyIdx++] = b;
      if (bodyIdx == (int)dataLen + 4) {
        bool trailerOk =
          body[dataLen]     == CMD_TRAILER[0] &&
          body[dataLen + 1] == CMD_TRAILER[1] &&
          body[dataLen + 2] == CMD_TRAILER[2] &&
          body[dataLen + 3] == CMD_TRAILER[3];
        return trailerOk;
      }
    }
  }

  return false;
}

int16_t RD03D::decodeSigned(uint8_t lo, uint8_t hi) {
  uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
  int16_t  mag = (int16_t)(raw & 0x7FFF);
  return (raw & 0x8000) ? (int16_t)(-mag) : mag;
}

bool RD03D::feedByte(uint8_t b) {
  switch (_state) {
    case WAIT_HEADER:
      if (b == HEADER[_headerIdx]) {
        _buf[_bufPos++] = b;
        if (++_headerIdx >= HEADER_SIZE) {
          _state = READ_DATA;
          _headerIdx = 0;
        }
      } else {
        _bufPos = 0;
        _headerIdx = 0;
        if (b == HEADER[0]) {
          _buf[_bufPos++] = b;
          _headerIdx = 1;
        }
      }
      return false;

    case READ_DATA:
      _buf[_bufPos++] = b;
      if (_bufPos >= FRAME_SIZE) {
        bool ok = (_buf[FRAME_SIZE - 2] == TAIL[0] &&
                   _buf[FRAME_SIZE - 1] == TAIL[1]);
        if (ok) {
          parseFrame();
          _framesOk++;
        } else {
          _framesBad++;
        }
        _bufPos = 0;
        _headerIdx = 0;
        _state = WAIT_HEADER;
        return ok;
      }
      return false;
  }
  return false;
}

void RD03D::parseFrame() {
  uint32_t now = millis();
  _targetCount = 0;

  for (uint8_t t = 0; t < MAX_TARGETS; t++) {
    uint8_t  off = HEADER_SIZE + t * TARGET_SIZE;

    int16_t  x   = decodeSigned(_buf[off],     _buf[off + 1]);
    int16_t  y   = decodeSigned(_buf[off + 2], _buf[off + 3]);
    int16_t  sp  = decodeSigned(_buf[off + 4], _buf[off + 5]);
    uint16_t dr  = (uint16_t)_buf[off + 6] | ((uint16_t)_buf[off + 7] << 8);

    Target &tg = _targets[t];
    tg.x = x;
    tg.y = y;
    tg.speed = sp;
    tg.distanceRes = dr;
    tg.timestamp = now;
    tg.valid = (dr != 0) || (x != 0) || (y != 0) || (sp != 0);

    if (tg.valid) {
      _targetCount++;
    }
  }
}
