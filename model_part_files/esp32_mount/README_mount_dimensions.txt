ESP32-S3 TWO-PART MOUNT V1
==========================

Purpose
-------
Two-piece removable mount for the dual-USB ESP32-S3 board discussed in chat.
Designed so the PCB sits face-up, female Dupont connectors can plug onto the
downward male pins, and wires can drop below the board before being routed.

IMPORTANT
---------
This V1 is based on the agreed nominal PCB envelope:
  Board length: 62.74 mm
  Board width : 25.40 mm
  PCB thickness assumption: 1.60 mm

Before committing the full robot top deck, measure the actual board with
calipers and test-print this mount.

MAIN DIMENSIONS
---------------
Overall mount footprint: 72.0 x 34.0 mm

Lower cradle:
  Base perimeter thickness: 2.0 mm
  Base frame wall width: 3.0 mm
  Pillar section: 5.5 x 5.5 mm
  Pillar top height above deck: 13.7 mm
  Nominal PCB underside height: 12.0 mm
  Nominal under-board wiring clearance: 12.0 mm
  PCB support bridge thickness: 2.0 mm
  Lower screw pilot holes: Ø1.7 mm (for M2 self-tapping)

Upper removable retainer:
  Overall size: 72.0 x 34.0 mm
  Thickness: 2.5 mm
  Main perimeter width: 3.0 mm
  USB-side central opening: 22.0 mm
  Screw clearance holes: Ø2.3 mm (M2)

Fasteners:
  4 x M2 screws recommended.
  Exact screw length depends on desired thread engagement; approximately
  8-10 mm is a sensible starting point for this geometry.

FILES
-----
esp32_s3_mount_lower_cradle_v1.stl
esp32_s3_mount_upper_retainer_v1.stl
esp32_s3_mount_lower_cradle_v1.step
esp32_s3_mount_upper_retainer_v1.step
esp32_s3_mount_assembly_v1.step

The STEP files are included because they will be much easier than STL to
modify/boolean into the future robot top deck.
