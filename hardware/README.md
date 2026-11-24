# Hardware Assembly

Open `notuchswitch.kicad_pro` in KiCad 9 or later. The PCB placement and
outline are fixed to fit the selected enclosure; do not move components.

## Required files

- `notuchswitch.kicad_sch`, `notuchswitch.kicad_pcb`, and
  `notuchswitch.kicad_pro`: schematic, PCB, and project.
- `BOM.csv`: component values, footprints, and parts that still need a
  dimensional check.
- `fabrication/gerber/`: copper, mask, silkscreen, outline, and drill files
  for PCB fabrication.
- `notuchswitch.pretty/`, `notuchswitch.kicad_sym`, `fp-lib-table`, and
  `sym-lib-table`: project-local footprints and symbol library configuration.
- `templates/cover_drill_template.pdf`: printable sensor and LED hole layout.

## Assembly notes

1. Order the PCB from the supplied Gerber and drill files, or export fresh
   fabrication files from the reviewed KiCad project.
2. Populate components according to `BOM.csv`. The fuse, varistor, and X2
   capacitor footprints are placeholders: verify the exact purchased parts,
   lead spacing, and body dimensions before soldering.
3. Confirm the APDS-9960 breakout pin order before inserting it. The board
   expects `VL, GND, VCC, SDA, SCL, INT`.
4. Solder the four marked 0.75 mm2 leads: brown `L_IN`, blue `N_IN`, black
   `L_SW`, and blue `N_OUT`. Keep the two neutral leads separately identified.
5. Relay pin 14 is brought out to solder pad `J8`. Connect it to fused live
   with an insulated external wire as shown by the schematic.
6. Do not tin stranded wire ends that will be clamped into screw terminals.
7. Mount the enclosure first, then fit the PCB. Print the cover template at
   100% and verify its 20 mm scale bar before drilling.

## Design details

- Two-layer, 1.0 mm FR4 PCB with a fixed octagonal outline.
- `HLK-2M05` isolated supply: 5 V, 400 mA continuous. Verify the load budget
  for the relay, LED, and the exact APDS breakout used.
- The recommended `10D561K` varistor may not match the supplied substitute;
  confirm its rating and dimensions against the supply manufacturer's data.
- Mains and low-voltage copper are separated by the project's 3 mm design rule.
- J7 is the on-board 2x2 SerialUPDI interface. R9 (470 ohm) and D5 (BAT54HT1G)
  are already fitted on the PCB design.
- USB-A provides 5 V only; D+ and D- are unconnected. Charging behavior is
  device-dependent and is not guaranteed.

The Gerber set is included as a fabrication artifact. Recheck the current KiCad
design, exact purchased components, and electrical safety before fabrication or
connecting mains. This prototype is not a certified product.
