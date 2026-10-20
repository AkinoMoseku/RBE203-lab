#ifndef ROBOT_BLUETOOTH_JOYSTICK_H
#define ROBOT_BLUETOOTH_JOYSTICK_H

#include <stdint.h>
#include <string.h>

// Tutorial protocol: [joystick,LH,LV,RH,RV], four integers in [-100,100].
// Only LV controls speed here. No String, dynamic allocation or blocking reads.
struct JoystickFrame { int8_t lh, lv, rh, rv; };

class JoystickFrameParser
{
public:
  void reset() { length = 0; receiving = false; }

  bool feed(char value, uint32_t now, JoystickFrame &frame)
  {
    if (receiving && (uint32_t)(now - lastByteMs) >= 200) reset();
    if (value == '[')
    {
      length = 0;
      receiving = true;
      lastByteMs = now;
      return false;
    }
    if (!receiving) return false;
    lastByteMs = now;
    if (value == ']')
    {
      text[length] = '\0';
      receiving = false;
      return parse(frame);
    }
    if (length >= sizeof(text) - 1 || value < 32 || value > 126)
    {
      reset();
      return false;
    }
    text[length++] = value;
    return false;
  }

private:
  char text[40] = {};
  uint8_t length = 0;
  bool receiving = false;
  uint32_t lastByteMs = 0;

  bool parse(JoystickFrame &frame)
  {
    if (strncmp(text, "joystick,", 9) != 0) return false;
    const char *cursor = text + 9;
    int8_t values[4];
    for (uint8_t i = 0; i < 4; ++i)
    {
      bool negative = false;
      if (*cursor == '-' || *cursor == '+') negative = *cursor++ == '-';
      if (*cursor < '0' || *cursor > '9') return false;
      uint16_t value = 0;
      uint8_t digits = 0;
      while (*cursor >= '0' && *cursor <= '9')
      {
        if (++digits > 3) return false;
        value = value * 10 + (*cursor++ - '0');
      }
      if (value > 100) return false;
      values[i] = negative ? -(int16_t)value : value;
      if (i < 3)
      {
        if (*cursor++ != ',') return false;
      }
      else if (*cursor != '\0') return false;
    }
    frame.lh = values[0]; frame.lv = values[1];
    frame.rh = values[2]; frame.rv = values[3];
    return true;
  }
};

class JoystickSpeedCommand
{
public:
  // Called from the 5 ms control callback. A transition invalidates queued data.
  bool tick(uint32_t now, bool blocked)
  {
    if (blocked != blockedByProtection)
    {
      blockedByProtection = blocked;
      clear();
      return true;
    }
    if (hasPacket && (uint32_t)(now - lastPacketMs) >= 500)
    {
      clear();
      return true;
    }
    return false;
  }

  void receive(int8_t lv, uint32_t now)
  {
    if (blockedByProtection) return;
    if (hasPacket && (uint32_t)(now - lastPacketMs) >= 500) clear();
    lastPacketMs = now;
    hasPacket = true;
    lastLv = lv;
    if (lv >= -5 && lv <= 5) needsCenter = false;
  }

  float target(float maximumSpeed, int8_t direction) const
  {
    if (blockedByProtection || !hasPacket || needsCenter) return 0;
    const int magnitude = lastLv < 0 ? -lastLv : lastLv;
    if (magnitude <= 5) return 0;
    const float speed = maximumSpeed * (magnitude - 5) / 95.0f;
    return (lastLv < 0 ? -speed : speed) * direction;
  }

  // 0=no fresh packet, 1=active, 2=needs neutral, 3=protection blocks motion.
  uint8_t state() const
  {
    if (blockedByProtection) return 3;
    if (!hasPacket) return 0;
    return needsCenter ? 2 : 1;
  }
  int8_t lv() const { return lastLv; }

private:
  bool blockedByProtection = true, hasPacket = false, needsCenter = true;
  int8_t lastLv = 0;
  uint32_t lastPacketMs = 0;

  void clear()
  {
    hasPacket = false;
    needsCenter = true;
    lastLv = 0;
  }
};

#endif
