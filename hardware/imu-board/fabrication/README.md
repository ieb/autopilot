# Fabrication Files

This folder contains manufacturing files for the IMU Sensor Fusion Board.

## Contents

- `BOM.csv` - Bill of Materials with LCSC part numbers for JLCPCB assembly
- `CPL.csv` - Component Placement List for pick-and-place assembly

## Generating Gerbers

Open `imu-board.kicad_pcb` in KiCad and:

1. **File → Plot** to open the plot dialog
2. Select layers:
   - F.Cu, B.Cu (copper)
   - F.Paste, B.Paste (solder paste)
   - F.SilkS, B.SilkS (silkscreen)
   - F.Mask, B.Mask (solder mask)
   - Edge.Cuts (board outline)
3. Set output directory to `fabrication/`
4. Check "Use Protel filename extensions"
5. Click **Plot**
6. Click **Generate Drill Files**
   - Select Excellon format
   - Click **Generate Drill File**

## JLCPCB Order

1. Zip the Gerber and drill files
2. Upload to [jlcpcb.com](https://jlcpcb.com)
3. Select options:
   - 2 layers
   - 1.6mm thickness
   - HASL lead-free finish
   - Green solder mask (or preferred color)
4. Enable SMT Assembly if desired
   - Upload `BOM.csv` and `CPL.csv`
   - Select "Assemble top side"

## Notes

- The ICM-20948 IMU module (J3) is not included in assembly - solder separately
- UPDI header (J2) uses standard 2.54mm pin headers
- Total board size: 20mm x 20mm
