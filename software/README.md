# Firmware

Open `notuchswitch_firmware/notuchswitch_firmware.ino` in Arduino IDE 2. The
sketch uses the adjacent `Apds9960Lite.cpp` and `Apds9960Lite.h` driver files;
keep all three in the same sketch folder.

## Arduino IDE setup

1. Add `http://drazzy.com/package_drazzy.com_index.json` under **Additional
   Boards Manager URLs** in Preferences.
2. Install **megaTinyCore** from Boards Manager.
3. Select the **ATtiny3216** chip and set **Clock** to `10 MHz internal`.
4. Set **BOD Voltage Level** to `2.6V`, **BOD Mode** to `Enabled/Enabled`,
   **Save EEPROM** to `EEPROM retained`, and **UPDI/Reset Pin Function** to
   `UPDI (no reset pin)`.
5. Select **SerialUPDI - 230400 baud** as the programmer.
6. Open the firmware sketch and use **Sketch > Verify/Compile**.
7. For programming and first-time fuse setup, follow [`SERIALUPDI.md`](SERIALUPDI.md).

## PCB pinout

| Function | ATtiny3216 port | SOIC-20 pin |
| --- | --- | ---: |
| Relay driver | PC0 | 12 |
| WS2812 status LED | PC1 | 13 |
| APDS-9960 SDA | PB1 | 10 |
| APDS-9960 SCL | PB0 | 11 |
| APDS-9960 interrupt | PA3 | 19 |
| UPDI | PA0 | 16 |

The firmware supports manual up/down gesture switching, simple door-presence
switching, a 45-minute auto-off for door-triggered activation, door-delta
adjustment, and automatic calibration. See the [user guide](../docs/user-guide.md)
for the LED menu.
