#include "Apds9960Lite.h"

namespace {

constexpr uint8_t kEnable = 0x80;
constexpr uint8_t kAtime = 0x81;
constexpr uint8_t kWtime = 0x83;
constexpr uint8_t kPpulse = 0x8E;
constexpr uint8_t kControl = 0x8F;
constexpr uint8_t kConfig2 = 0x90;
constexpr uint8_t kId = 0x92;
constexpr uint8_t kPdata = 0x9C;
constexpr uint8_t kGpent = 0xA0;
constexpr uint8_t kGexth = 0xA1;
constexpr uint8_t kGconf1 = 0xA2;
constexpr uint8_t kGconf2 = 0xA3;
constexpr uint8_t kGoffsetU = 0xA4;
constexpr uint8_t kGoffsetD = 0xA5;
constexpr uint8_t kGpulse = 0xA6;
constexpr uint8_t kGoffsetL = 0xA7;
constexpr uint8_t kGoffsetR = 0xA9;
constexpr uint8_t kGconf3 = 0xAA;
constexpr uint8_t kGconf4 = 0xAB;
constexpr uint8_t kGflvl = 0xAE;
constexpr uint8_t kGstatus = 0xAF;
constexpr uint8_t kGfifoU = 0xFC;

constexpr uint8_t kEnablePon = 0x01;
constexpr uint8_t kEnablePen = 0x04;
constexpr uint8_t kEnableWen = 0x08;
constexpr uint8_t kEnableGen = 0x40;
constexpr uint8_t kGvalid = 0x01;
// SparkFun's reference gesture init uses LED_BOOST_300 in CONFIG2. Proximity
// can work without it, but the gesture FIFO is much more sensitive with boost.
constexpr uint8_t kConfig2LedBoost300 = 0x31;
// Fixed near-range gesture detection. The previous entry threshold of 15
// triggered the gesture FIFO too far away, even at 25-30 cm through the cover.
// These values are closer to the APDS-9960 reference settings and target
// manual gestures at roughly 10 cm.
constexpr uint8_t kGestureEnterThreshold = 45;
constexpr uint8_t kGestureExitThreshold = 30;
constexpr uint8_t kI2cRetryCount = 3;
constexpr uint8_t kInitRetryCount = 5;
constexpr uint16_t kI2cRetryDelayMs = 2;
constexpr uint16_t kInitRetryDelayMs = 80;

}  // namespace

bool Apds9960Lite::begin() {
  for (uint8_t attempt = 0; attempt < kInitRetryCount; ++attempt) {
    Wire.swap(0);
    Wire.begin();
    Wire.setClock(100000);
    delay(kInitRetryDelayMs);

    uint8_t id = 0;
    if (read8(kId, id) &&
        write8(kEnable, 0x00) &&
        write8(kAtime, 0xDB) &&
        write8(kWtime, 0xFF) &&
        write8(kPpulse, 0x89) &&
        write8(kControl, 0x09) &&
        write8(kConfig2, kConfig2LedBoost300) &&
        setGestureEnterThreshold(kGestureEnterThreshold) &&
        write8(kGexth, kGestureExitThreshold) &&
        write8(kGconf1, 0x40) &&
        write8(kGconf2, 0x41) &&
        write8(kGoffsetU, 0) &&
        write8(kGoffsetD, 0) &&
        write8(kGoffsetL, 0) &&
        write8(kGoffsetR, 0) &&
        write8(kGpulse, 0xC9) &&
        write8(kGconf3, 0x00) &&
        write8(kGconf4, 0x03) &&
        write8(kEnable, kEnablePon | kEnablePen | kEnableWen | kEnableGen)) {
      resetGesture();
      return true;
    }

    write8(kEnable, 0x00);
    delay(kInitRetryDelayMs);
  }

  return false;
}

bool Apds9960Lite::readProximity(uint8_t &value) {
  return read8(kPdata, value);
}

bool Apds9960Lite::setGestureEnterThreshold(uint8_t threshold) {
  return write8(kGpent, threshold);
}

Apds9960Lite::Gesture Apds9960Lite::pollGesture(uint32_t nowMs) {
  uint8_t status = 0;
  if (!read8(kGstatus, status)) {
    return Gesture::None;
  }

  if ((status & kGvalid) != 0) {
    uint8_t fifoLevel = 0;
    if (!read8(kGflvl, fifoLevel) || fifoLevel == 0) {
      return Gesture::None;
    }

    const uint8_t samples =
        min(fifoLevel, static_cast<uint8_t>(kMaxSamplesPerRead));
    uint8_t fifo[kMaxSamplesPerRead * 4] = {};
    if (!readBlock(kGfifoU, fifo, samples * 4)) {
      return Gesture::None;
    }

    processFifo(fifo, samples);
    gestureActive_ = true;
    lastFifoMs_ = nowMs;
    if (validSamples_ >= kMinGestureSamples) {
      return finishGesture();
    }
    return Gesture::None;
  }

  if (gestureActive_ &&
      static_cast<uint32_t>(nowMs - lastFifoMs_) >= kGestureQuietMs) {
    return finishGesture();
  }

  return Gesture::None;
}

void Apds9960Lite::resetGesture() {
  gestureActive_ = false;
  validSamples_ = 0;
  firstUdRatio_ = 0;
  firstLrRatio_ = 0;
  lastUdRatio_ = 0;
  lastLrRatio_ = 0;
  lastFifoMs_ = 0;
}

bool Apds9960Lite::write8(uint8_t reg, uint8_t value) {
  for (uint8_t attempt = 0; attempt < kI2cRetryCount; ++attempt) {
    Wire.beginTransmission(kAddress);
    Wire.write(reg);
    Wire.write(value);
    if (Wire.endTransmission() == 0) {
      return true;
    }
    delay(kI2cRetryDelayMs);
  }
  return false;
}

bool Apds9960Lite::read8(uint8_t reg, uint8_t &value) {
  for (uint8_t attempt = 0; attempt < kI2cRetryCount; ++attempt) {
    Wire.beginTransmission(kAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) == 0 &&
        Wire.requestFrom(kAddress, static_cast<uint8_t>(1)) == 1) {
      value = Wire.read();
      return true;
    }
    delay(kI2cRetryDelayMs);
  }
  return false;
}

bool Apds9960Lite::readBlock(uint8_t reg, uint8_t *data, uint8_t length) {
  for (uint8_t attempt = 0; attempt < kI2cRetryCount; ++attempt) {
    Wire.beginTransmission(kAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) == 0 &&
        Wire.requestFrom(kAddress, length) == length) {
      for (uint8_t i = 0; i < length; ++i) {
        data[i] = Wire.read();
      }
      return true;
    }
    delay(kI2cRetryDelayMs);
  }
  return false;
}

void Apds9960Lite::processFifo(const uint8_t *fifo, uint8_t sampleCount) {
  for (uint8_t i = 0; i < sampleCount; ++i) {
    const uint8_t up = fifo[i * 4];
    const uint8_t down = fifo[i * 4 + 1];
    const uint8_t left = fifo[i * 4 + 2];
    const uint8_t right = fifo[i * 4 + 3];

    if (up <= kMinFifoValue || down <= kMinFifoValue ||
        left <= kMinFifoValue || right <= kMinFifoValue) {
      continue;
    }

    const int16_t udRatio =
        static_cast<int16_t>((static_cast<int32_t>(up - down) * 100) /
                             (up + down));
    const int16_t lrRatio =
        static_cast<int16_t>((static_cast<int32_t>(left - right) * 100) /
                             (left + right));

    if (validSamples_ == 0) {
      firstUdRatio_ = udRatio;
      firstLrRatio_ = lrRatio;
    }

    lastUdRatio_ = udRatio;
    lastLrRatio_ = lrRatio;
    if (validSamples_ < 255) {
      ++validSamples_;
    }
  }
}

Apds9960Lite::Gesture Apds9960Lite::finishGesture() {
  if (validSamples_ < kMinGestureSamples) {
    resetGesture();
    return Gesture::None;
  }

  const int16_t udDelta = lastUdRatio_ - firstUdRatio_;
  const int16_t lrDelta = lastLrRatio_ - firstLrRatio_;
  resetGesture();

  const int16_t udMagnitude = abs(udDelta);
  const int16_t lrMagnitude = abs(lrDelta);

  if (udDelta > 0 && udMagnitude >= kDownDirectionThreshold &&
      udMagnitude >= lrMagnitude + kDownAxisSeparationThreshold) {
    return Gesture::Down;
  }

  if (udMagnitude < kDirectionThreshold && lrMagnitude < kDirectionThreshold) {
    return Gesture::None;
  }

  if (abs(udMagnitude - lrMagnitude) < kAxisSeparationThreshold) {
    return Gesture::None;
  }

  if (udMagnitude > lrMagnitude) {
    return udDelta < 0 ? Gesture::Up : Gesture::Down;
  }

  return lrDelta < 0 ? Gesture::Left : Gesture::Right;
}
