# ATtiny3216 Programming

The ATtiny3216 uses UPDI. A classic USBasp is an AVR ISP programmer and cannot
connect directly to the board's J7 header. Use a 3.3 V logic USB-to-UART
adapter (for example, CH340, CP2102, or FT232) and the SerialUPDI programmer
provided by megaTinyCore. The UPDI resistor and Schottky diode are already on
the PCB.

## J7 pinout

J7 is a 2x2, 2.54 mm header. Pin 1 is the square pad, viewed from the component
side of the PCB:

```text
1 UART_RX | 2 UART_TX
3 +3V3    | 4 GND
```

| USB-to-UART adapter | Board J7 |
| --- | --- |
| RX | Pin 1, UART_RX |
| TX | Pin 2, UART_TX |
| 3.3 V | Pin 3, +3V3 |
| GND | Pin 4, GND |

R9 (470 ohm) is in series between UART_RX and the MCU UPDI pin. D5
(BAT54HT1G) connects UART_RX to UART_TX, with its cathode toward UART_TX.

## Arduino IDE setup and upload

1. Install megaTinyCore using the Boards Manager URL in [`README.md`](README.md).
2. Select the ATtiny3216 board and settings listed in [`README.md`](README.md).
3. Select **SerialUPDI - 230400 baud** as the programmer. If uploads are
   unreliable, use **SerialUPDI - SLOW: 57600 baud**.
4. Connect the adapter to J7 and select its serial port in Arduino IDE.
5. Run **Tools > Burn Bootloader** once to set the fuse bits. This setting
   does not install a bootloader.
6. Open `notuchswitch_firmware/notuchswitch_firmware.ino`, then use
   **Sketch > Upload Using Programmer**.

Keep the board disconnected from mains while programming. J7 pin 3 powers the
low-voltage control circuit. If the adapter cannot supply about 150 mA at
3.3 V, use a regulated external source with a shared ground. Do not connect
two 3.3 V sources to J7 pin 3 at once.

References: [megaTinyCore](https://github.com/SpenceKonde/megaTinyCore),
[SerialUPDI guide](https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/extras/Ref_SerialUPDI.md),
[ATtiny3216](https://www.microchip.com/en-us/product/attiny3216).
