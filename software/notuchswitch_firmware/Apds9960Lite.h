#pragma once

#include <Arduino.h>
#include <Wire.h>

class Apds9960Lite {
 public:
  enum class Gesture : uint8_t {
    None,
    Up,
    Down,
    Left,
    Right,
  };

  bool begin();
  bool readProximity(uint8_t &value);
  bool setGestureEnterThreshold(uint8_t threshold);
  Gesture pollGesture(uint32_t nowMs);
  void resetGesture();

 private:
  static constexpr uint8_t kAddress = 0x39;
  static constexpr uint8_t kMaxSamplesPerRead = 8;
  static constexpr uint8_t kMinGestureSamples = 4;
  static constexpr uint8_t kMinFifoValue = 6;
  static constexpr int16_t kDirectionThreshold = 28;
  static constexpr int16_t kDownDirectionThreshold = 22;
  static constexpr int16_t kAxisSeparationThreshold = 15;
  static constexpr int16_t kDownAxisSeparationThreshold = 8;
  static constexpr uint16_t kGestureQuietMs = 60;

  bool write8(uint8_t reg, uint8_t value);
  bool read8(uint8_t reg, uint8_t &value);
  bool readBlock(uint8_t reg, uint8_t *data, uint8_t length);
  void processFifo(const uint8_t *fifo, uint8_t sampleCount);
  Gesture finishGesture();

  bool gestureActive_ = false;
  uint8_t validSamples_ = 0;
  int16_t firstUdRatio_ = 0;
  int16_t firstLrRatio_ = 0;
  int16_t lastUdRatio_ = 0;
  int16_t lastLrRatio_ = 0;
  uint32_t lastFifoMs_ = 0;
};
