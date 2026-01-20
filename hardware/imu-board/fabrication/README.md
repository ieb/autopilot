# IMU Board Fabrication

Instructions for manufacturing the IMU Sensor Fusion Board.

## PCB Specifications

| Parameter | Value |
|-----------|-------|
| Dimensions | 20mm × 20mm |
| Layers | 2 |
| Thickness | 1.6mm |
| Copper Weight | 1oz (35µm) |
| Min Track Width | 0.2mm (8mil) |
| Min Clearance | 0.2mm (8mil) |
| Min Via Drill | 0.3mm |
| Surface Finish | HASL or ENIG |
| Solder Mask | Green |
| Silkscreen | White |

## Generating Gerber Files

1. Open `imu-board.kicad_pcb` in KiCad
2. File → Plot
3. Select layers:
   - F.Cu (Front Copper)
   - B.Cu (Back Copper)
   - F.SilkS (Front Silkscreen)
   - B.SilkS (Back Silkscreen)
   - F.Mask (Front Solder Mask)
   - B.Mask (Back Solder Mask)
   - Edge.Cuts (Board Outline)
4. Set output directory: `fabrication/gerbers/`
5. Click "Plot"
6. Click "Generate Drill Files"
   - Format: Excellon
   - Map File Format: Gerber X2

## Ordering from JLCPCB

### PCB Only

1. Go to [jlcpcb.com](https://jlcpcb.com)
2. Click "Quote Now"
3. Upload Gerber ZIP file
4. Verify dimensions detected correctly (20mm × 20mm)
5. Select options:
   - Layers: 2
   - PCB Thickness: 1.6mm
   - PCB Color: Green
   - Surface Finish: HASL(with lead) or LeadFree HASL
   - Remove Order Number: Specify a location
6. Add to cart and checkout

### PCB + Assembly (SMT)

1. Enable "SMT Assembly"
2. Select:
   - Assemble top side
   - PCBA Qty: as needed
   - Tooling holes: Added by JLCPCB
3. Upload BOM file: `BOM.csv`
4. Upload CPL file: `CPL.csv`
5. Review component placement in viewer
6. Confirm parts availability (check LCSC stock)
7. Add to cart and checkout

## Component Placement Notes

The CPL (Component Placement List) coordinates assume:
- Origin at bottom-left corner of board
- Board is 20mm × 20mm
- All SMD components on top layer

Adjust coordinates if your PCB origin differs.

## Through-Hole Components

J2 (UPDI programming header) is through-hole and must be soldered manually after SMT assembly.

## External Components

The ICM-20948 IMU module is not included on the PCB. Purchase separately:

| Vendor | Part Number | Notes |
|--------|-------------|-------|
| SparkFun | SEN-15335 | Qwiic connector, easiest |
| Adafruit | 4554 | STEMMA QT connector |
| Generic | - | Check pinout matches |

Connect module to J3 using a 4-wire JST-SH cable.

## Quality Checks

After receiving boards:

1. **Visual Inspection**
   - Check solder joints under magnification
   - Verify component orientation (U1, U2 pin 1)
   - Check for solder bridges

2. **Electrical Test**
   - Apply 5V, verify 3.3V output
   - Check no shorts between VCC and GND
   - Measure quiescent current (~1mA without IMU)

3. **Functional Test**
   - Program with firmware
   - Connect IMU module
   - Verify serial output

## Files Checklist

```
fabrication/
├── BOM.csv           ✓ Bill of Materials (LCSC format)
├── CPL.csv           ✓ Component Placement List
├── README.md         ✓ This file
└── gerbers/          □ Generate from KiCad (not tracked in git)
    ├── imu-board-F_Cu.gbr
    ├── imu-board-B_Cu.gbr
    ├── imu-board-F_SilkS.gbr
    ├── imu-board-B_SilkS.gbr
    ├── imu-board-F_Mask.gbr
    ├── imu-board-B_Mask.gbr
    ├── imu-board-Edge_Cuts.gbr
    └── imu-board.drl
```

## Revision History

| Rev | Date | Changes |
|-----|------|---------|
| 1.0 | 2026-01-20 | Initial fabrication files |
