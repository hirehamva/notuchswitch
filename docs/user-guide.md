# User Guide

## Normal operation

- Move a hand upward in front of the sensor to toggle the lamp immediately.
- A downward gesture also toggles the lamp, but is delayed by about 1.8 seconds
  while the firmware waits to see whether it is the start of the menu-entry code.
- Move the door or another object steadily into the sensor's field of view to
  toggle the lamp. Moving it away only rearms detection; it does not switch the
  lamp by itself.
- Keep the sensor clear before the next door movement. Continuous obstruction
  cannot trigger repeated switching.
- After a manual turn-on, the door is ignored until the next manual turn-off.
- The optional 45-minute auto-off applies only to a door-triggered turn-on.
  It is enabled by default; manual turn-ons do not start the timer.

## Status LED

| LED indication | Meaning |
| --- | --- |
| Off | Lamp is off |
| Dim green | Lamp is on |
| Red blinking | Sensor initialization or I2C error; the firmware retries every five seconds |
| Brief cyan after startup | Sensor initialization succeeded |
| Blue flash | Up gesture recognized |
| Orange flash | Down gesture recognized |
| Cyan flash | Left gesture recognized |
| Purple flash | Right gesture recognized |

Direction-colour acknowledgements are enabled in the current prototype
firmware. Horizontal gestures are diagnostic only during normal operation.

## Opening the settings menu

Perform three downward gestures. Allow no more than about 1.8 seconds between
gestures:

```text
DOWN  +  DOWN  +  DOWN  ->  settings menu
```

After entering, move your hand away once. This arms the long-hold save/exit
gesture and prevents the hand used to enter the menu from immediately closing it.

| Gesture | Menu action |
| --- | --- |
| Left | Previous item |
| Right | Next item |
| Up | Increase, enable, or start |
| Down | Decrease or disable |
| Hold an object near the sensor for about 2.2 seconds | Save changes and exit |

The selected menu item is identified by its number of flashes. The flash group
repeats approximately every 2.2 seconds. The menu also exits after 30 seconds
without activity.

## Menu items

| Flashes | Item colour | Function | Up | Down |
| ---: | --- | --- | --- | --- |
| 1 | Purple | 45-minute auto-off | Enable | Disable |
| 2 | Orange | Door sensitivity | Increase delta; make less sensitive | Decrease delta; make more sensitive |
| 3 | Cyan | Automatic calibration | Start calibration | No action |

### 1 flash: 45-minute auto-off

This timer applies only when the door turns the lamp on. When enabled, the lamp
turns off 45 minutes after a door-triggered activation. Turning the lamp on
manually never starts the timer. If the lamp is already on due to the door and
you enable auto-off in the menu, the timer starts when you enable it.

After changing the setting, green means enabled and red means disabled.

### 2 flashes: door sensitivity

Door detection compares current proximity with the learned unobstructed
baseline. The stored door delta is the increase required to detect an
obstruction. Its default is 35 and the adjustable range is 15 to 120.

1. Select the orange two-flash item with left/right gestures.
2. If the door triggers from too far away or reacts to noise, gesture up to
   increase the delta and reduce sensitivity.
3. If the door does not always trigger, gesture down to decrease the delta and
   increase sensitivity.
4. Red acknowledges adjustment toward lower sensitivity; green acknowledges
   adjustment toward higher sensitivity. Brightness increases toward the
   corresponding end of the range, and the indication is dimmer near the middle.
5. White indicates that the current value is at a range limit.
6. Test the door after each change. The controller relearns the clear baseline
   as needed.
7. Hold near the sensor for about 2.2 seconds to save and exit, or wait for the
   30-second menu timeout.

An automatically calibrated delta can also be fine-tuned here. Detection
distance is not a direct centimetre setting and varies with the optical and
mechanical installation.

### 3 flashes: automatic calibration

Run calibration after the sensor and enclosure have been installed in their
final positions. It measures both the clear sensor and the door in its normal
obstructing position.

1. Select the cyan three-flash item.
2. Gesture up to start calibration.
3. During the blue phase, remove your hand and keep the sensor area clear. The
   controller measures the unobstructed baseline.
4. During the yellow phase, move the door into the position where it normally
   covers the sensor.
5. During the cyan phase, keep the door still while proximity is sampled.
6. Green means calibration succeeded. The calculated delta is saved immediately.
7. Red means calibration failed. The door may not have reached the detected
   position within 30 seconds, or the clear and obstructed readings may not
   have been far enough apart.
8. After the result indication, the menu returns to the calibration item. Hold
   near the sensor for about 2.2 seconds to exit.

## Enclosure and sensor notes

The APDS-9960 gesture engine and proximity detector can be affected by the
cover opening, wall thickness, sensor setback, and reflections inside the
enclosure. A narrow or rough opening can distort gesture directions. The door
function uses proximity rather than gesture direction, so it can continue to
work even when the cover affects directional gestures.

The cover drill template is in
[`hardware/templates/cover_drill_template.pdf`](../hardware/templates/cover_drill_template.pdf).
Print it at 100% scale and verify the 20 mm scale bar before drilling. The
template is based on a SparkFun-compatible APDS-9960 breakout; verify the exact
module geometry before using it.

## Troubleshooting

| Symptom | Likely cause | Check |
| --- | --- | --- |
| Red LED blinking | APDS-9960 initialization or I2C error | Supply, SDA/SCL/INT wiring, breakout pin order, and module jumpers |
| Door triggers too far away | Door delta is too low | Increase the delta in the two-flash sensitivity menu |
| Door does not trigger reliably | Door delta is too high or the optical contrast is weak | Decrease the delta or run calibration in the final enclosure |
| Gesture directions change with the cover fitted | The opening is clipping or reflecting infrared light | Check sensor alignment, opening size, edge finish, and sensor setback |
| USBasp cannot upload | USBasp provides ISP, while this board uses UPDI | Use a 3.3 V USB-to-UART adapter with SerialUPDI on J7 |

## Prototype safety

The controller switches mains voltage. Disconnect mains power before opening,
adjusting, or servicing the enclosure. This project is an engineering
prototype, not a certified mains product; installation and electrical safety
must be reviewed by a qualified person.
