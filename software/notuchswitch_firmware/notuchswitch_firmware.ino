#include <Arduino.h>
#include <EEPROM.h>
#include <tinyNeoPixel_Static.h>

#include "Apds9960Lite.h"

/*
 * Firmware for an ATtiny3216-based no-touch light switch.
 *
 * The firmware coordinates three independent tasks:
 *
 * 1. Normal operation:
 *    - vertical hand gestures switch the lamp manually;
 *    - each new, stable proximity event from the door toggles the lamp;
 *    - horizontal gestures receive diagnostic acknowledgements only;
 *    - an optional 45-minute auto-off timer follows a door-triggered turn-on.
 *
 * 2. Gesture menu:
 *    - three consecutive downward gestures open it;
 *    - left and right select menu items;
 *    - up and down change values;
 *    - a long sensor hold saves changes and exits.
 *
 * 3. Automatic calibration:
 *    - first measure the unobstructed sensor baseline;
 *    - then measure the door in its detected position;
 *    - calculate the door delta from both averages and save it to EEPROM.
 *
 * loop() remains non-blocking and uses millis()-based timing, allowing the
 * sensor, relay, LED feedback, and menu to run concurrently.
 */

namespace {

// Final PCB pin assignments. Change these only when the hardware changes.
constexpr uint8_t kRelayPin = PIN_PC0;
constexpr uint8_t kStatusLedPin = PIN_PC1;
constexpr uint8_t kApdsInterruptPin = PIN_PA3;

// Timing values in milliseconds.
constexpr uint32_t kAutoOffMs = 45UL * 60UL * 1000UL;
// Poll the gesture FIFO on a timer as well as from the interrupt path so a
// missed interrupt does not prevent gesture processing.
constexpr uint32_t kGesturePollMs = 12;
// Proximity can be polled more slowly for menu and calibration tasks.
constexpr uint32_t kProximityPollMs = 40;
// Ignore new gestures for this interval to prevent duplicate recognition.
constexpr uint32_t kGestureCooldownMs = 500;
// Gesture acknowledgement duration; long enough to identify direction while
// tuning the prototype.
constexpr uint32_t kGestureAcknowledgementMs = 400;
// Show the updated value colour for this long after a menu adjustment. The
// longer interval helps distinguish subtle brightness changes through the cover.
constexpr uint32_t kMenuValueAcknowledgementMs = 1200;
// Long acknowledgement flash for saving and exiting the menu.
constexpr uint32_t kSaveAcknowledgementMs = 900;
// Also identify the active menu item by its pulse count.
constexpr uint32_t kMenuPulseOnMs = 180;
constexpr uint32_t kMenuPulseOffMs = 130;
constexpr uint32_t kMenuBlinkCycleMs = 2200;
constexpr uint32_t kMenuBlinkRefreshMs = 40;
// Continuous proximity duration required to exit the menu.
constexpr uint32_t kMenuHoldMs = 2200;
// Menu inactivity timeout during normal operation.
constexpr uint32_t kMenuTimeoutMs = 30000;
// Maximum allowed gap between the three downward menu-entry gestures.
constexpr uint32_t kMenuDownGestureGapMs = 1800;
// Allow time for the hand to leave the sensor before sampling the clear
// baseline during calibration.
constexpr uint32_t kCalibrationSettleMs = 1000;
// Maximum wait for the door after measuring the clear baseline.
constexpr uint32_t kCalibrationDoorTimeoutMs = 30000;
// Duration of the calibration success or failure indication.
constexpr uint32_t kCalibrationResultMs = 1400;
// Retry sensor initialization at this interval after a sensor error.
constexpr uint32_t kSensorRetryMs = 5000;
// Red LED half-period while reporting a sensor error.
constexpr uint32_t kErrorBlinkMs = 500;
// The APDS module may start more slowly than the ATtiny. A short startup delay
// reduces false I2C errors during power-up.
constexpr uint16_t kSensorStartupDelayMs = 350;

// Limits for manually adjusted and automatically calibrated proximity values.
// Although the APDS-9960 register is 8-bit, the full 0...255 range is not used.
constexpr uint8_t kThresholdMin = 20;
constexpr uint8_t kThresholdMax = 200;
constexpr uint8_t kDefaultProximityThreshold = 30;
// Door presence uses the increase from the unobstructed baseline rather than
// an absolute proximity value. This accommodates the enclosure, sensor spacing,
// and mechanical variation. The assembled prototype detected objects too far
// away, so the delta range was raised to target operation at roughly 10 cm.
constexpr uint8_t kSimpleDoorMinDelta = 15;
constexpr uint8_t kSimpleDoorMaxDelta = 120;
constexpr uint8_t kDefaultDoorDeltaThreshold = 35;
constexpr uint8_t kDoorDeltaStep = 1;
// Require this many consecutive proximity samples to detect and release an
// object, filtering isolated noise and passing shadows.
constexpr uint8_t kSimpleDoorStableSamples = 3;
// Number of downward gestures required to open the menu.
constexpr uint8_t kMenuDownGestureCount = 3;
// Average 24 samples at 40 ms intervals, or approximately one second of data.
constexpr uint8_t kCalibrationSampleCount = 24;
// Calibration starts sampling the door after proximity rises this far above
// the clear baseline.
constexpr uint8_t kCalibrationDoorTriggerDelta = 4;
// Accept calibration only when the two measured states are clearly separated.
constexpr uint8_t kCalibrationMinDifference = 4;
// EEPROM identifier and format version. Increment the version when Settings
// changes layout.
constexpr uint16_t kSettingsMagic = 0x4E54;
constexpr uint8_t kSettingsVersion = 4;
// false: acknowledge all normal-operation gestures in white.
// true: show direction colours during normal operation for development.
// Calibration always uses direction colours regardless of this setting.
constexpr bool kUseDirectionalGestureAcknowledgement = true;

// WS2812 brightness levels. Coloured acknowledgements peak at about 50%
// (128/255); the steady green status uses about 25% (64/255).
constexpr uint8_t kLedHigh = 128;
constexpr uint8_t kLedHalf = 64;
constexpr uint8_t kLedGreen = 64;
constexpr uint8_t kLedLow = 24;

// Valid ranges for legacy EEPROM fields. The user menu no longer uses these,
// but they remain so old settings can be read safely.
enum class DoorDirection : uint8_t {
  FromLeft = 0,
  FromRight = 1,
};

enum class ManualMode : uint8_t {
  VerticalToggle = 0,
  UpOnDownOff = 1,
};

enum class ControlMode : uint8_t {
  Smart = 0,
  Simple = 1,
};

// The relay activation source matters: after a manual turn-on, ignore door
// events until the next manual turn-off.
enum class ActivationSource : uint8_t {
  None,
  Door,
  Manual,
};

// Count is a sentinel for wrapping menu navigation, not an actual menu item.
enum class MenuItem : uint8_t {
  AutoOff = 0,
  ProximityThreshold,
  Calibration,
  Count,
};

// Calibration is non-blocking. Each proximity sample advances this state
// machine by at most one step.
enum class CalibrationPhase : uint8_t {
  // No calibration is in progress.
  Idle,
  // Wait for the user to move their hand away after starting.
  WaitingForClear,
  // Average the unobstructed sensor baseline.
  SamplingClear,
  // Wait for the door to enter the sensor's field of view.
  WaitingForDoor,
  // Average the proximity value with the door in position.
  SamplingDoor,
  // Accepted result: brief green indication.
  Success,
  // Rejected result or timeout: brief red indication.
  Failure,
};

// User settings stored in EEPROM. The packed attribute prevents the compiler
// from inserting padding bytes between fields.
struct __attribute__((packed)) Settings {
  uint16_t magic;
  uint8_t version;
  uint8_t doorDirection;
  uint8_t autoOffEnabled;
  uint8_t manualMode;
  uint8_t controlMode;
  uint8_t proximityThreshold;
  uint8_t doorDeltaThreshold;
  uint8_t checksum;
};

// Loading also recognizes the previous v1 and v2 layouts, then migrates them
// to v4 with fixed simple mode and a door delta.
struct __attribute__((packed)) SettingsV1 {
  uint16_t magic;
  uint8_t version;
  uint8_t doorDirection;
  uint8_t autoOffEnabled;
  uint8_t manualMode;
  uint8_t proximityThreshold;
  uint8_t checksum;
};

struct __attribute__((packed)) SettingsV2 {
  uint16_t magic;
  uint8_t version;
  uint8_t doorDirection;
  uint8_t autoOffEnabled;
  uint8_t manualMode;
  uint8_t controlMode;
  uint8_t proximityThreshold;
  uint8_t checksum;
};

// tinyNeoPixel requires external RAM storage for the LED's three colour bytes.
uint8_t ledPixels[3] = {};
tinyNeoPixel statusLed(1, kStatusLedPin, NEO_GRB, ledPixels);
Apds9960Lite sensor;
Settings settings = {};

// The interrupt sets this flag, so it is volatile. I2C communication runs in
// loop(), never inside the interrupt handler.
volatile bool sensorInterruptPending = false;

// General runtime state.
bool sensorReady = false;
bool lampOn = false;
// True after a manual turn-on: ignore presence-based door events until the
// next manual turn-off.
bool manualOverrideActive = false;
bool menuActive = false;
// Write EEPROM on menu exit only if a setting actually changed.
bool menuDirty = false;
// Do not redraw the base status colour while an acknowledgement is active.
bool gestureAcknowledgementActive = false;
// Prevent one continuous sensor hold from triggering multiple menu exits.
bool nearActionDone = false;
// Require the sensor to clear once after menu entry; otherwise the hand that
// opened the menu could immediately trigger the hold-to-exit action.
bool menuHoldArmed = false;
bool errorLedOn = false;
bool simpleDoorObjectPresent = false;
bool simpleDoorSuppressedUntilClear = false;
bool simpleDoorBaselineValid = false;
uint8_t simpleDoorBaseline = 0;
uint8_t simpleDoorNearSamples = 0;
uint8_t simpleDoorReleaseSamples = 0;

// Current state-machine phases.
MenuItem menuItem = MenuItem::AutoOff;
CalibrationPhase calibrationPhase = CalibrationPhase::Idle;

// Counters and intermediate calibration results.
uint8_t downGestureCount = 0;
uint8_t calibrationSamples = 0;
uint8_t calibrationClearAverage = 0;
uint16_t calibrationAccumulator = 0;

// millis()-based deadlines and timestamps of the last events.
uint32_t autoOffDeadlineMs = 0;
uint32_t gestureAcknowledgementDeadlineMs = 0;
uint32_t calibrationDeadlineMs = 0;
uint32_t nearStartMs = 0;
uint32_t lastDownGestureMs = 0;
uint32_t lastGesturePollMs = 0;
uint32_t lastProximityPollMs = 0;
uint32_t lastGestureHandledMs = 0;
uint32_t lastMenuActivityMs = 0;
uint32_t menuBlinkStartMs = 0;
uint32_t lastMenuBlinkDrawMs = 0;
uint32_t lastSensorRetryMs = 0;
uint32_t lastErrorBlinkMs = 0;

void renderNormalStatus();
void renderMenuStatus();
void updateMenuBlink(uint32_t nowMs);
void startCalibration(uint32_t nowMs);
void updateCalibration(uint8_t proximity, uint32_t nowMs);
void resetSimpleDoorPresenceCounters();
void resetSimpleDoorBaseline();

// Simple XOR checksum. This is not cryptographic protection; it only detects
// uninitialized or corrupted EEPROM data.
uint8_t settingsChecksum(const Settings &value) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);
  uint8_t checksum = 0xA5;
  for (uint8_t i = 0; i < sizeof(Settings) - 1; ++i) {
    checksum ^= bytes[i];
  }
  return checksum;
}

uint8_t settingsV1Checksum(const SettingsV1 &value) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);
  uint8_t checksum = 0xA5;
  for (uint8_t i = 0; i < sizeof(SettingsV1) - 1; ++i) {
    checksum ^= bytes[i];
  }
  return checksum;
}

uint8_t settingsV2Checksum(const SettingsV2 &value) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);
  uint8_t checksum = 0xA5;
  for (uint8_t i = 0; i < sizeof(SettingsV2) - 1; ++i) {
    checksum ^= bytes[i];
  }
  return checksum;
}

bool settingsV1AreValid(const SettingsV1 &value) {
  return value.magic == kSettingsMagic && value.version == 1 &&
         value.doorDirection <=
             static_cast<uint8_t>(DoorDirection::FromRight) &&
         value.autoOffEnabled <= 1 &&
         value.manualMode <= static_cast<uint8_t>(ManualMode::UpOnDownOff) &&
         value.proximityThreshold >= kThresholdMin &&
         value.proximityThreshold <= kThresholdMax &&
         value.checksum == settingsV1Checksum(value);
}

bool settingsV2AreValid(const SettingsV2 &value) {
  return value.magic == kSettingsMagic && value.version == 2 &&
         value.doorDirection <=
             static_cast<uint8_t>(DoorDirection::FromRight) &&
         value.autoOffEnabled <= 1 &&
         value.manualMode <= static_cast<uint8_t>(ManualMode::UpOnDownOff) &&
         value.controlMode <= static_cast<uint8_t>(ControlMode::Simple) &&
         value.proximityThreshold >= kThresholdMin &&
         value.proximityThreshold <= kThresholdMax &&
         value.checksum == settingsV2Checksum(value);
}

bool settingsAreValid(const Settings &value) {
  return value.magic == kSettingsMagic &&
         value.version == kSettingsVersion &&
         value.doorDirection <=
             static_cast<uint8_t>(DoorDirection::FromRight) &&
         value.autoOffEnabled <= 1 &&
         value.manualMode <= static_cast<uint8_t>(ManualMode::UpOnDownOff) &&
         value.controlMode <= static_cast<uint8_t>(ControlMode::Simple) &&
         value.proximityThreshold >= kThresholdMin &&
         value.proximityThreshold <= kThresholdMax &&
         value.doorDeltaThreshold >= kSimpleDoorMinDelta &&
         value.doorDeltaThreshold <= kSimpleDoorMaxDelta &&
         value.checksum == settingsChecksum(value);
}

// EEPROM.update() writes a cell only when its value changes, preserving the
// EEPROM's limited write endurance.
void saveSettings() {
  settings.magic = kSettingsMagic;
  settings.version = kSettingsVersion;
  settings.checksum = settingsChecksum(settings);

  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&settings);
  for (uint8_t i = 0; i < sizeof(Settings); ++i) {
    EEPROM.update(i, bytes[i]);
  }
}

uint8_t clampDoorDelta(int16_t value) {
  return constrain(value, kSimpleDoorMinDelta, kSimpleDoorMaxDelta);
}

uint8_t defaultDoorDeltaFromProximity(uint8_t /*proximityThreshold*/) {
  return kDefaultDoorDeltaThreshold;
}

// Older firmware may leave smart mode, door direction, or manual style in
// EEPROM. Force these to fixed values at startup so the device always uses
// simple presence-based logic.
bool normalizeRetiredSettings() {
  bool changed = false;

  if (settings.doorDirection !=
      static_cast<uint8_t>(DoorDirection::FromLeft)) {
    settings.doorDirection = static_cast<uint8_t>(DoorDirection::FromLeft);
    changed = true;
  }

  if (settings.manualMode !=
      static_cast<uint8_t>(ManualMode::VerticalToggle)) {
    settings.manualMode = static_cast<uint8_t>(ManualMode::VerticalToggle);
    changed = true;
  }

  if (settings.controlMode != static_cast<uint8_t>(ControlMode::Simple)) {
    settings.controlMode = static_cast<uint8_t>(ControlMode::Simple);
    changed = true;
  }

  const uint8_t clampedDelta = clampDoorDelta(settings.doorDeltaThreshold);
  if (settings.doorDeltaThreshold != clampedDelta) {
    settings.doorDeltaThreshold = clampedDelta;
    changed = true;
  }

  return changed;
}

// Load valid EEPROM data at startup; otherwise save safe defaults. The relay
// starts turned off either way.
void loadSettings() {
  EEPROM.get(0, settings);
  if (settingsAreValid(settings)) {
    if (normalizeRetiredSettings()) {
      saveSettings();
    }
    return;
  }

  SettingsV2 legacyV2 = {};
  EEPROM.get(0, legacyV2);
  if (settingsV2AreValid(legacyV2)) {
    settings.magic = kSettingsMagic;
    settings.version = kSettingsVersion;
    settings.doorDirection = static_cast<uint8_t>(DoorDirection::FromLeft);
    settings.autoOffEnabled = legacyV2.autoOffEnabled;
    settings.manualMode = static_cast<uint8_t>(ManualMode::VerticalToggle);
    settings.controlMode = static_cast<uint8_t>(ControlMode::Simple);
    settings.proximityThreshold = legacyV2.proximityThreshold;
    settings.doorDeltaThreshold =
        defaultDoorDeltaFromProximity(legacyV2.proximityThreshold);
    saveSettings();
    return;
  }

  SettingsV1 legacy = {};
  EEPROM.get(0, legacy);
  if (settingsV1AreValid(legacy)) {
    settings.magic = kSettingsMagic;
    settings.version = kSettingsVersion;
    settings.doorDirection = static_cast<uint8_t>(DoorDirection::FromLeft);
    settings.autoOffEnabled = legacy.autoOffEnabled;
    settings.manualMode = static_cast<uint8_t>(ManualMode::VerticalToggle);
    settings.controlMode = static_cast<uint8_t>(ControlMode::Simple);
    settings.proximityThreshold = legacy.proximityThreshold;
    settings.doorDeltaThreshold =
        defaultDoorDeltaFromProximity(legacy.proximityThreshold);
    saveSettings();
    return;
  }

  settings.magic = kSettingsMagic;
  settings.version = kSettingsVersion;
  settings.doorDirection = static_cast<uint8_t>(DoorDirection::FromLeft);
  settings.autoOffEnabled = 1;
  settings.manualMode = static_cast<uint8_t>(ManualMode::VerticalToggle);
  settings.controlMode = static_cast<uint8_t>(ControlMode::Simple);
  settings.proximityThreshold = kDefaultProximityThreshold;
  settings.doorDeltaThreshold = kDefaultDoorDeltaThreshold;
  saveSettings();
}

// Set the single WS2812 LED. The kLed* constants limit brightness so it is
// not dazzling through the enclosure.
void setStatusLed(uint8_t red, uint8_t green, uint8_t blue) {
  statusLed.setPixelColor(0, statusLed.Color(red, green, blue));
  statusLed.show();
}

uint8_t activeMenuPulseCount() {
  return static_cast<uint8_t>(menuItem) + 1;
}

bool menuPulseIsOn(uint32_t nowMs) {
  const uint32_t elapsed =
      static_cast<uint32_t>(nowMs - menuBlinkStartMs) % kMenuBlinkCycleMs;
  const uint32_t pulseWindow = kMenuPulseOnMs + kMenuPulseOffMs;
  const uint32_t activeWindow =
      static_cast<uint32_t>(activeMenuPulseCount()) * pulseWindow;

  if (elapsed >= activeWindow) {
    return false;
  }

  return (elapsed % pulseWindow) < kMenuPulseOnMs;
}

void restartMenuBlink(uint32_t nowMs) {
  menuBlinkStartMs = nowMs;
  lastMenuBlinkDrawMs = 0;
}

void startSaveAcknowledgement(uint32_t nowMs) {
  gestureAcknowledgementActive = true;
  gestureAcknowledgementDeadlineMs = nowMs + kSaveAcknowledgementMs;
  setStatusLed(kLedHigh, kLedHigh, kLedHigh);
}

// A longer flash acknowledges every accepted gesture.
//
// During tuning, normal operation also uses direction colours so it is clear
// which direction the APDS-9960 recognized.
void startGestureAcknowledgement(Apds9960Lite::Gesture gesture,
                                 uint32_t nowMs) {
  gestureAcknowledgementActive = true;
  gestureAcknowledgementDeadlineMs = nowMs + kGestureAcknowledgementMs;

  if (!kUseDirectionalGestureAcknowledgement &&
      calibrationPhase == CalibrationPhase::Idle) {
    setStatusLed(kLedHigh, kLedHigh, kLedHigh);
    return;
  }

  switch (gesture) {
    case Apds9960Lite::Gesture::Up:
      setStatusLed(0, 0, kLedHigh);
      break;
    case Apds9960Lite::Gesture::Down:
      setStatusLed(kLedHigh, kLedHalf, 0);
      break;
    case Apds9960Lite::Gesture::Left:
      setStatusLed(0, kLedHigh, kLedHigh);
      break;
    case Apds9960Lite::Gesture::Right:
      setStatusLed(kLedHigh, 0, kLedHigh);
      break;
    case Apds9960Lite::Gesture::None:
      break;
  }
}

// After the flash, redraw the current base state: lamp status in normal
// operation or the active menu item in the menu.
void updateGestureAcknowledgement(uint32_t nowMs) {
  if (!gestureAcknowledgementActive ||
      static_cast<int32_t>(nowMs - gestureAcknowledgementDeadlineMs) < 0) {
    return;
  }

  gestureAcknowledgementActive = false;
  if (menuActive) {
    restartMenuBlink(nowMs);
    renderMenuStatus();
  } else {
    renderNormalStatus();
  }
}

// Central relay controller. Route every relay change through this function
// to keep manual override and auto-off behavior in one place.
void setRelay(bool enabled, ActivationSource source) {
  lampOn = enabled;
  digitalWrite(kRelayPin, enabled ? HIGH : LOW);
  // Only a manual turn-on temporarily disables door control.
  manualOverrideActive = enabled && source == ActivationSource::Manual;

  // Start the auto-off timer only when the door turned the lamp on.
  if (enabled && source == ActivationSource::Door &&
      settings.autoOffEnabled != 0) {
    autoOffDeadlineMs = millis() + kAutoOffMs;
  } else {
    autoOffDeadlineMs = 0;
  }

  if (!menuActive) {
    renderNormalStatus();
  }
}

// In normal operation, dark means off and green means on. Sensor errors and
// gesture acknowledgements use their own LED patterns.
void renderNormalStatus() {
  if (!sensorReady || gestureAcknowledgementActive) {
    return;
  }

  if (lampOn) {
    setStatusLed(0, kLedGreen, 0);
  } else {
    setStatusLed(0, 0, 0);
  }
}

// During calibration, the base LED colour identifies the current phase:
// blue = clear the sensor or measure the unobstructed baseline,
// yellow = wait for the door to reach its detected position,
// cyan = sample proximity with the door in position,
// green = calibration succeeded,
// red = calibration failed.
void renderCalibrationStatus() {
  switch (calibrationPhase) {
    case CalibrationPhase::WaitingForClear:
    case CalibrationPhase::SamplingClear:
      setStatusLed(0, 0, kLedHigh);
      break;

    case CalibrationPhase::WaitingForDoor:
      setStatusLed(kLedHigh, kLedHalf, 0);
      break;

    case CalibrationPhase::SamplingDoor:
      setStatusLed(0, kLedHigh, kLedHigh);
      break;

    case CalibrationPhase::Success:
      setStatusLed(0, kLedGreen, 0);
      break;

    case CalibrationPhase::Failure:
      setStatusLed(kLedHigh, 0, 0);
      break;

    case CalibrationPhase::Idle:
      break;
  }
}

// Each menu item has a distinct base colour. Where useful, brightness or hue
// also indicates the current value. Gesture acknowledgements and calibration
// take priority over the menu colour.
void renderMenuStatus() {
  if (gestureAcknowledgementActive) {
    return;
  }

  if (calibrationPhase != CalibrationPhase::Idle) {
    renderCalibrationStatus();
    return;
  }

  if (!menuPulseIsOn(millis())) {
    setStatusLed(0, 0, 0);
    return;
  }

  switch (menuItem) {
    case MenuItem::AutoOff:
      setStatusLed(kLedHigh, 0, kLedHigh);
      break;

    case MenuItem::ProximityThreshold: {
      setStatusLed(kLedHigh, kLedHalf, 0);
      break;
    }

    case MenuItem::Calibration:
      setStatusLed(0, kLedHigh, kLedHigh);
      break;

    case MenuItem::Count:
      break;
  }
}

void updateMenuBlink(uint32_t nowMs) {
  if (!menuActive || calibrationPhase != CalibrationPhase::Idle ||
      gestureAcknowledgementActive) {
    return;
  }

  if (static_cast<uint32_t>(nowMs - lastMenuBlinkDrawMs) <
      kMenuBlinkRefreshMs) {
    return;
  }

  lastMenuBlinkDrawMs = nowMs;
  renderMenuStatus();
}

// After an up/down adjustment, show the new setting rather than the gesture
// direction. These colours intentionally differ from the fixed menu colours.
void renderMenuValueStatus() {
  switch (menuItem) {
    case MenuItem::AutoOff:
      if (settings.autoOffEnabled != 0) {
        setStatusLed(0, kLedGreen, 0);
      } else {
        setStatusLed(kLedHigh, 0, 0);
      }
      break;

    case MenuItem::ProximityThreshold: {
      const uint8_t range = kSimpleDoorMaxDelta - kSimpleDoorMinDelta;
      const uint8_t delta = clampDoorDelta(settings.doorDeltaThreshold);
      const uint8_t position = delta - kSimpleDoorMinDelta;
      const uint8_t fade =
          static_cast<uint8_t>((static_cast<uint16_t>(position) *
                                (kLedHigh - kLedLow)) /
                               range);
      const uint8_t red = kLedLow + fade;
      const uint8_t green = kLedHigh - fade;
      setStatusLed(red, green, 0);
      break;
    }

    case MenuItem::Calibration:
      setStatusLed(0, kLedHigh, kLedHigh);
      break;

    case MenuItem::Count:
      break;
  }
}

void startMenuValueAcknowledgement(uint32_t nowMs) {
  gestureAcknowledgementActive = true;
  gestureAcknowledgementDeadlineMs = nowMs + kMenuValueAcknowledgementMs;
  renderMenuValueStatus();
}

uint8_t doorDeltaAcknowledgementBrightness(bool increase) {
  const uint8_t delta = clampDoorDelta(settings.doorDeltaThreshold);
  const uint8_t midpoint =
      (kSimpleDoorMinDelta + kSimpleDoorMaxDelta) / 2;
  uint8_t distance = 0;
  uint8_t maxDistance = 1;

  if (increase) {
    // Red grows brighter from the midpoint toward the less-sensitive limit.
    if (delta > midpoint) {
      distance = delta - midpoint;
    }
    maxDistance = kSimpleDoorMaxDelta - midpoint;
  } else {
    // Green grows brighter from the midpoint toward the more-sensitive limit.
    if (delta < midpoint) {
      distance = midpoint - delta;
    }
    maxDistance = midpoint - kSimpleDoorMinDelta;
  }

  return kLedLow + static_cast<uint8_t>(
                       (static_cast<uint16_t>(distance) *
                        (kLedHigh - kLedLow)) /
                       maxDistance);
}

void startDoorDeltaAcknowledgement(bool increase, bool changed,
                                   uint32_t nowMs) {
  gestureAcknowledgementActive = true;
  gestureAcknowledgementDeadlineMs = nowMs + kMenuValueAcknowledgementMs;

  const uint8_t delta = clampDoorDelta(settings.doorDeltaThreshold);
  if (!changed) {
    setStatusLed(kLedHigh, kLedHigh, kLedHigh);
  } else if (increase) {
    if (delta >= kSimpleDoorMaxDelta) {
      setStatusLed(kLedHigh, kLedHigh, kLedHigh);
    } else {
      // A larger delta is less sensitive and is indicated in red.
      setStatusLed(doorDeltaAcknowledgementBrightness(increase), 0, 0);
    }
  } else {
    if (delta <= kSimpleDoorMinDelta) {
      setStatusLed(kLedHigh, kLedHigh, kLedHigh);
    } else {
      // A smaller delta is more sensitive and is indicated in green.
      setStatusLed(0, doorDeltaAcknowledgementBrightness(increase), 0);
    }
  }
}

// Always start at the first menu item. Disarm hold-to-exit until the sensor
// has cleared once, so the user must release it after entering the menu.
void enterMenu(uint32_t nowMs) {
  menuActive = true;
  menuDirty = false;
  menuHoldArmed = false;
  nearActionDone = false;
  nearStartMs = 0;
  calibrationPhase = CalibrationPhase::Idle;
  downGestureCount = 0;
  menuItem = MenuItem::AutoOff;
  lastMenuActivityMs = nowMs;
  restartMenuBlink(nowMs);
  renderMenuStatus();
}

// Save changes once on normal menu exit. Exiting during calibration cancels
// the operation and discards partial samples.
void leaveMenu(uint32_t nowMs) {
  if (menuDirty) {
    saveSettings();
  }
  calibrationPhase = CalibrationPhase::Idle;
  calibrationAccumulator = 0;
  calibrationSamples = 0;
  menuActive = false;
  menuHoldArmed = false;
  nearActionDone = false;
  nearStartMs = 0;
  simpleDoorObjectPresent = false;
  simpleDoorSuppressedUntilClear = true;
  simpleDoorNearSamples = 0;
  simpleDoorReleaseSamples = 0;
  resetSimpleDoorBaseline();
  startSaveAcknowledgement(nowMs);
}

// Wrap menu navigation from the first item to the last.
void selectPreviousMenuItem() {
  uint8_t value = static_cast<uint8_t>(menuItem);
  if (value == 0) {
    value = static_cast<uint8_t>(MenuItem::Count) - 1;
  } else {
    --value;
  }
  menuItem = static_cast<MenuItem>(value);
}

// Wrap menu navigation from the last item to the first.
void selectNextMenuItem() {
  uint8_t value = static_cast<uint8_t>(menuItem) + 1;
  if (value >= static_cast<uint8_t>(MenuItem::Count)) {
    value = 0;
  }
  menuItem = static_cast<MenuItem>(value);
}

// Up passes increase=true and down passes increase=false. At the calibration
// item, only an up gesture starts the process.
bool changeMenuValue(bool increase, uint32_t nowMs) {
  switch (menuItem) {
    case MenuItem::AutoOff:
      settings.autoOffEnabled = increase ? 1 : 0;
      if (settings.autoOffEnabled == 0) {
        autoOffDeadlineMs = 0;
      } else if (lampOn && !manualOverrideActive) {
        autoOffDeadlineMs = nowMs + kAutoOffMs;
      }
      break;

    case MenuItem::ProximityThreshold: {
      // Larger delta means less-sensitive door detection; smaller means more sensitive.
      const uint8_t previousDelta = settings.doorDeltaThreshold;
      const int16_t adjustment = increase ? kDoorDeltaStep : -kDoorDeltaStep;
      const int16_t nextDelta =
          static_cast<int16_t>(settings.doorDeltaThreshold) + adjustment;
      settings.doorDeltaThreshold = clampDoorDelta(nextDelta);
      resetSimpleDoorBaseline();
      const bool changed = settings.doorDeltaThreshold != previousDelta;
      menuDirty = menuDirty || changed;
      startDoorDeltaAcknowledgement(increase, changed, nowMs);
      return true;
    }

    case MenuItem::Calibration:
      if (increase) {
        startCalibration(nowMs);
      }
      return false;

    case MenuItem::Count:
      return false;
  }

  menuDirty = true;
  startMenuValueAcknowledgement(nowMs);
  return true;
}

// Clear the previous sample set before starting a new average.
void resetCalibrationSamples() {
  calibrationAccumulator = 0;
  calibrationSamples = 0;
}

// Add one proximity sample to the current calibration average. Return true
// when the full set is collected and the average parameter contains the result.
bool addCalibrationSample(uint8_t proximity, uint8_t &average) {
  calibrationAccumulator += proximity;
  ++calibrationSamples;

  if (calibrationSamples < kCalibrationSampleCount) {
    return false;
  }

  average = static_cast<uint8_t>(calibrationAccumulator / calibrationSamples);
  resetCalibrationSamples();
  return true;
}

// Subtraction remains correct across millis() rollover when deadlines are
// less than about 24 days away; all deadlines here are much shorter.
bool deadlineReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

// Start sampling the door when proximity rises this far above the clear
// baseline. Clamp to 255 because APDS-9960 proximity is an 8-bit value.
uint16_t calibrationDoorTrigger() {
  const uint16_t trigger =
      static_cast<uint16_t>(calibrationClearAverage) +
      kCalibrationDoorTriggerDelta;
  return trigger > 255 ? 255 : trigger;
}

// Show the result in a distinct colour briefly. Calibration remains active
// during this indication, so the normal menu timeout does not run.
void finishCalibration(bool success, uint32_t nowMs) {
  calibrationPhase =
      success ? CalibrationPhase::Success : CalibrationPhase::Failure;
  calibrationDeadlineMs = nowMs + kCalibrationResultMs;
  lastMenuActivityMs = nowMs;
  renderMenuStatus();
}

// The initiating up-swipe may still be in front of the sensor. Wait for
// kCalibrationSettleMs before taking the first measurement.
void startCalibration(uint32_t nowMs) {
  resetCalibrationSamples();
  calibrationClearAverage = 0;
  calibrationPhase = CalibrationPhase::WaitingForClear;
  calibrationDeadlineMs = nowMs + kCalibrationSettleMs;
  lastMenuActivityMs = nowMs;
  menuHoldArmed = false;
  nearActionDone = false;
  nearStartMs = 0;
  restartMenuBlink(nowMs);
  renderMenuStatus();
}

// Non-blocking automatic calibration state machine. It is called for each
// new proximity sample and performs at most one short step per call.
//
// On success, derive the door delta directly from the clear and door averages.
// Simple mode detects obstruction when baseline + delta is reached.
void updateCalibration(uint8_t proximity, uint32_t nowMs) {
  uint8_t average = 0;

  switch (calibrationPhase) {
    case CalibrationPhase::Idle:
      return;

    case CalibrationPhase::WaitingForClear:
      // Begin measurement after the hand-clear settling interval.
      if (deadlineReached(nowMs, calibrationDeadlineMs)) {
        calibrationPhase = CalibrationPhase::SamplingClear;
        resetCalibrationSamples();
        renderMenuStatus();
      }
      return;

    case CalibrationPhase::SamplingClear:
      // Wait for the door after the clear-baseline average is complete.
      if (addCalibrationSample(proximity, average)) {
        calibrationClearAverage = average;
        calibrationPhase = CalibrationPhase::WaitingForDoor;
        calibrationDeadlineMs = nowMs + kCalibrationDoorTimeoutMs;
        renderMenuStatus();
      }
      return;

    case CalibrationPhase::WaitingForDoor:
      // Show a red error on timeout. Include the first door sample as soon as
      // the door is detected.
      if (deadlineReached(nowMs, calibrationDeadlineMs)) {
        finishCalibration(false, nowMs);
      } else if (proximity >= calibrationDoorTrigger()) {
        calibrationPhase = CalibrationPhase::SamplingDoor;
        resetCalibrationSamples();
        addCalibrationSample(proximity, average);
        renderMenuStatus();
      }
      return;

    case CalibrationPhase::SamplingDoor:
      // If the door leaves during sampling, discard partial samples and wait
      // again so the average represents a stable position.
      if (deadlineReached(nowMs, calibrationDeadlineMs)) {
        finishCalibration(false, nowMs);
      } else if (proximity < calibrationDoorTrigger()) {
        calibrationPhase = CalibrationPhase::WaitingForDoor;
        resetCalibrationSamples();
        renderMenuStatus();
      } else if (addCalibrationSample(proximity, average)) {
        if (average <= calibrationClearAverage) {
          finishCalibration(false, nowMs);
          return;
        }

        const uint16_t difference =
            static_cast<uint16_t>(average - calibrationClearAverage);
        if (difference < kCalibrationMinDifference) {
          finishCalibration(false, nowMs);
          return;
        }

        // Half the measured difference is a compromise: detect the door early
        // while retaining noise margin above the clear state.
        settings.doorDeltaThreshold =
            clampDoorDelta(static_cast<int16_t>((difference + 1) / 2));
        resetSimpleDoorBaseline();
        saveSettings();
        menuDirty = false;
        finishCalibration(true, nowMs);
      }
      return;

    case CalibrationPhase::Success:
    case CalibrationPhase::Failure:
      // Return to the same menu item after the result indication ends.
      if (deadlineReached(nowMs, calibrationDeadlineMs)) {
        calibrationPhase = CalibrationPhase::Idle;
        menuItem = MenuItem::Calibration;
        lastMenuActivityMs = nowMs;
        menuHoldArmed = false;
        restartMenuBlink(nowMs);
        renderMenuStatus();
      }
      return;
  }
}

void resetSimpleDoorPresenceCounters() {
  simpleDoorNearSamples = 0;
  simpleDoorReleaseSamples = 0;
}

void resetSimpleDoorBaseline() {
  simpleDoorBaselineValid = false;
  simpleDoorBaseline = 0;
}

uint8_t simpleDoorDeltaThreshold() {
  return clampDoorDelta(settings.doorDeltaThreshold);
}

uint8_t addClamped(uint8_t base, uint8_t delta) {
  const uint16_t value = static_cast<uint16_t>(base) + delta;
  return value > 255 ? 255 : static_cast<uint8_t>(value);
}

uint8_t simpleDoorEnterLevel() {
  return addClamped(simpleDoorBaseline, simpleDoorDeltaThreshold());
}

uint8_t simpleDoorReleaseLevel() {
  const uint8_t delta = simpleDoorDeltaThreshold();
  const uint8_t releaseDelta = delta <= 2 ? 1 : delta / 2;
  return addClamped(simpleDoorBaseline, releaseDelta);
}

void updateSimpleDoorBaseline(uint8_t proximity) {
  if (!simpleDoorBaselineValid) {
    simpleDoorBaseline = proximity;
    simpleDoorBaselineValid = true;
    return;
  }

  if (proximity < simpleDoorBaseline) {
    simpleDoorBaseline = static_cast<uint8_t>(
        (static_cast<uint16_t>(simpleDoorBaseline) * 3 + proximity + 2) / 4);
  } else {
    simpleDoorBaseline = static_cast<uint8_t>(
        (static_cast<uint16_t>(simpleDoorBaseline) * 15 + proximity + 8) /
        16);
  }
}

void ensureSimpleDoorBaseline(uint8_t proximity) {
  if (!simpleDoorBaselineValid) {
    simpleDoorBaseline = proximity;
    simpleDoorBaselineValid = true;
  }
}

void suppressSimpleDoorUntilClear() {
  simpleDoorSuppressedUntilClear = true;
  resetSimpleDoorPresenceCounters();
}

void handleSimpleDoorPresenceEvent() {
  if (manualOverrideActive) {
    return;
  }

  if (lampOn) {
    setRelay(false, ActivationSource::None);
  } else {
    setRelay(true, ActivationSource::Door);
  }
}

void updateSimpleDoorPresence(uint8_t proximity) {
  if (menuActive || calibrationPhase != CalibrationPhase::Idle) {
    return;
  }

  ensureSimpleDoorBaseline(proximity);

  const bool near = proximity >= simpleDoorEnterLevel();
  const bool released = proximity <= simpleDoorReleaseLevel();
  const bool clearForArming = !near;

  if (simpleDoorObjectPresent) {
    if (released) {
      ++simpleDoorReleaseSamples;
      if (simpleDoorReleaseSamples >= kSimpleDoorStableSamples) {
        simpleDoorObjectPresent = false;
        simpleDoorSuppressedUntilClear = false;
        resetSimpleDoorPresenceCounters();
        updateSimpleDoorBaseline(proximity);
      }
    } else {
      simpleDoorReleaseSamples = 0;
    }
    return;
  }

  if (simpleDoorSuppressedUntilClear) {
    if (clearForArming) {
      ++simpleDoorReleaseSamples;
      if (simpleDoorReleaseSamples < kSimpleDoorStableSamples) {
        return;
      }
      simpleDoorSuppressedUntilClear = false;
      resetSimpleDoorPresenceCounters();
      if (released) {
        updateSimpleDoorBaseline(proximity);
      }
    } else {
      simpleDoorReleaseSamples = 0;
    }
    return;
  }

  if (!near) {
    simpleDoorNearSamples = 0;
    if (released) {
      updateSimpleDoorBaseline(proximity);
    }
    return;
  }

  ++simpleDoorNearSamples;
  if (simpleDoorNearSamples < kSimpleDoorStableSamples) {
    return;
  }

  simpleDoorObjectPresent = true;
  resetSimpleDoorPresenceCounters();
  handleSimpleDoorPresenceEvent();
}

// Manual control uses vertical gestures only. Up and down both toggle the
// relay; down is delayed to allow the three-gesture menu code.
void applyManualGesture(Apds9960Lite::Gesture /*gesture*/) {
  setRelay(!lampOn, ActivationSource::Manual);
}

// Discard a partial sequence of downward gestures for menu entry.
void resetDownGestureSequence() {
  downGestureCount = 0;
  lastDownGestureMs = 0;
}

// A downward gesture has two meanings:
// - one gesture toggles the lamp in normal operation;
// - three rapid gestures open the menu.
//
// Therefore, defer a single downward gesture. If the previous gesture is too
// old, execute it before starting a new sequence.
void registerDownGesture(uint32_t nowMs) {
  if (downGestureCount != 0 &&
      static_cast<uint32_t>(nowMs - lastDownGestureMs) >
          kMenuDownGestureGapMs) {
    applyManualGesture(Apds9960Lite::Gesture::Down);
    resetDownGestureSequence();
  }

  ++downGestureCount;
  lastDownGestureMs = nowMs;

  if (downGestureCount >= kMenuDownGestureCount) {
    resetDownGestureSequence();
    enterMenu(nowMs);
  }
}

// If no further downward gesture arrives within the allowed interval, treat
// the collected sequence as one normal manual downward toggle.
void updatePendingDownGesture(uint32_t nowMs) {
  if (!menuActive && downGestureCount != 0 &&
      static_cast<uint32_t>(nowMs - lastDownGestureMs) >
          kMenuDownGestureGapMs) {
    applyManualGesture(Apds9960Lite::Gesture::Down);
    resetDownGestureSequence();
  }
}

// Route normal-operation gestures by function: down is a delayed manual
// toggle or menu code, up toggles immediately, and left/right provide only
// direction diagnostics. Door switching requires stable proximity obstruction,
// so the door moving away cannot trigger a switch.
void handleNormalGesture(Apds9960Lite::Gesture gesture, uint32_t nowMs) {
  if (gesture == Apds9960Lite::Gesture::Down) {
    suppressSimpleDoorUntilClear();
    registerDownGesture(nowMs);
    return;
  }

  resetDownGestureSequence();
  if (gesture == Apds9960Lite::Gesture::Up) {
    suppressSimpleDoorUntilClear();
    applyManualGesture(gesture);
    return;
  }
}

// In the menu, left and right select items while up and down change values.
// During calibration, gestures receive coloured diagnostic acknowledgements
// only; they cannot change settings or interrupt measurements.
bool handleMenuGesture(Apds9960Lite::Gesture gesture, uint32_t nowMs) {
  lastMenuActivityMs = nowMs;

  if (calibrationPhase != CalibrationPhase::Idle) {
    return false;
  }

  switch (gesture) {
    case Apds9960Lite::Gesture::Left:
      selectPreviousMenuItem();
      restartMenuBlink(nowMs);
      renderMenuStatus();
      return false;

    case Apds9960Lite::Gesture::Right:
      selectNextMenuItem();
      restartMenuBlink(nowMs);
      renderMenuStatus();
      return false;

    case Apds9960Lite::Gesture::Up:
      return changeMenuValue(true, nowMs);

    case Apds9960Lite::Gesture::Down:
      return changeMenuValue(false, nowMs);

    case Apds9960Lite::Gesture::None:
      return false;
  }

  return false;
}

// Shared gesture entry point. The cooldown filters duplicate detections from
// one hand movement. Start LED feedback after processing so ignored door-mode
// gestures can still be seen.
void handleGesture(Apds9960Lite::Gesture gesture, uint32_t nowMs) {
  if (gesture == Apds9960Lite::Gesture::None ||
      static_cast<uint32_t>(nowMs - lastGestureHandledMs) <
          kGestureCooldownMs) {
    return;
  }

  lastGestureHandledMs = nowMs;
  bool acknowledgementStarted = false;
  if (menuActive) {
    acknowledgementStarted = handleMenuGesture(gesture, nowMs);
  } else {
    handleNormalGesture(gesture, nowMs);
  }
  if (!acknowledgementStarted) {
    startGestureAcknowledgement(gesture, nowMs);
  }
}

// Save and exit the menu by holding an object near the sensor.
//
// Arm menuHoldArmed only after the sensor has first returned to a clear value
// following menu entry. This prevents immediate accidental exit. Hold-to-exit
// is disabled during calibration because door proximity is not an exit command.
void updateNearHold(uint8_t proximity, uint32_t nowMs) {
  if (!menuActive || calibrationPhase != CalibrationPhase::Idle) {
    menuHoldArmed = false;
    nearStartMs = 0;
    nearActionDone = false;
    return;
  }

  const bool near = proximity >= settings.proximityThreshold;
  if (!near) {
    nearStartMs = 0;
    nearActionDone = false;
    menuHoldArmed = true;
    return;
  }

  if (!menuHoldArmed) {
    return;
  }

  if (nearStartMs == 0) {
    nearStartMs = nowMs;
  }

  if (!nearActionDone &&
      static_cast<uint32_t>(nowMs - nearStartMs) >= kMenuHoldMs) {
    nearActionDone = true;
    leaveMenu(nowMs);
  }
}

// Keep the hardware interrupt minimal: it only tells loop() to read the
// APDS-9960 gesture FIFO soon.
void sensorInterrupt() {
  sensorInterruptPending = true;
}

// Initialize the APDS-9960. Keep the relay off if initialization fails.
bool initialiseSensor(uint32_t nowMs) {
  lastSensorRetryMs = nowMs;
  sensorReady = sensor.begin();
  if (sensorReady) {
    sensorInterruptPending = true;
    errorLedOn = false;
    renderNormalStatus();
  } else {
    setRelay(false, ActivationSource::None);
  }
  return sensorReady;
}

// With a working sensor, handle two data streams:
// - poll the gesture FIFO frequently to capture quick hand movements;
// - poll proximity less often for menu and calibration tasks.
void updateSensor(uint32_t nowMs) {
  if (sensorInterruptPending ||
      static_cast<uint32_t>(nowMs - lastGesturePollMs) >= kGesturePollMs) {
    sensorInterruptPending = false;
    lastGesturePollMs = nowMs;
    handleGesture(sensor.pollGesture(nowMs), nowMs);
  }

  if (static_cast<uint32_t>(nowMs - lastProximityPollMs) >=
      kProximityPollMs) {
    lastProximityPollMs = nowMs;
    uint8_t proximity = 0;
    if (sensor.readProximity(proximity)) {
      const bool wasMenuOrCalibration =
          menuActive || calibrationPhase != CalibrationPhase::Idle;
      updateCalibration(proximity, nowMs);
      updateNearHold(proximity, nowMs);
      if (!wasMenuOrCalibration) {
        updateSimpleDoorPresence(proximity);
      }
    }
  }
}

// Sensor-error mode retries initialization on a timer while blinking red, so
// the firmware can recover automatically from a temporary I2C error.
void updateSensorFailure(uint32_t nowMs) {
  if (static_cast<uint32_t>(nowMs - lastErrorBlinkMs) >= kErrorBlinkMs) {
    lastErrorBlinkMs = nowMs;
    errorLedOn = !errorLedOn;
    setStatusLed(errorLedOn ? kLedHigh : 0, 0, 0);
  }

  if (static_cast<uint32_t>(nowMs - lastSensorRetryMs) >= kSensorRetryMs) {
    initialiseSensor(nowMs);
  }
}

// The optional safety timer turns off a lamp that was switched on by the door.
// A manual turn-on does not start the timer.
void updateAutoOff(uint32_t nowMs) {
  if (autoOffDeadlineMs != 0 &&
      static_cast<int32_t>(nowMs - autoOffDeadlineMs) >= 0) {
    setRelay(false, ActivationSource::None);
  }
}

}  // namespace

// Runs once after power-up or reset.
void setup() {
  // Configure the relay pin as an output and drive it LOW first, preventing
  // an accidental turn-on during initialization.
  pinMode(kRelayPin, OUTPUT);
  digitalWrite(kRelayPin, LOW);

  // Keep the status LED dark at startup.
  pinMode(kStatusLedPin, OUTPUT);
  statusLed.begin();
  setStatusLed(0, 0, 0);

  // The APDS-9960 interrupt is active-low. The ISR only sets a flag; I2C reads
  // happen during normal loop() execution.
  pinMode(kApdsInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kApdsInterruptPin), sensorInterrupt,
                  FALLING);

  // Load user settings before normal operation begins.
  loadSettings();
  simpleDoorSuppressedUntilClear = true;
  resetSimpleDoorPresenceCounters();
  resetSimpleDoorBaseline();
  delay(kSensorStartupDelayMs);
  if (initialiseSensor(millis())) {
    setStatusLed(0, kLedHigh, kLedHigh);
    delay(350);
    renderNormalStatus();
  }
}

// The main loop runs short, non-blocking tasks in this deliberate order:
// 1. handle sensor data or sensor errors;
// 2. process menu inactivity exit;
// 3. execute a delayed downward gesture;
// 4. check auto-off;
// 5. expire LED acknowledgements;
// 6. update the menu blink pattern.
void loop() {
  const uint32_t nowMs = millis();

  if (sensorReady) {
    updateSensor(nowMs);
  } else {
    updateSensorFailure(nowMs);
  }

  // Do not apply the menu inactivity timeout during calibration; the user
  // needs time to move the door into its detected position.
  if (menuActive && calibrationPhase == CalibrationPhase::Idle &&
      static_cast<uint32_t>(nowMs - lastMenuActivityMs) >= kMenuTimeoutMs) {
    leaveMenu(nowMs);
  }

  updatePendingDownGesture(nowMs);
  updateAutoOff(nowMs);
  updateGestureAcknowledgement(nowMs);
  updateMenuBlink(nowMs);
}
