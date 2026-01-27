# Kraken Drive + NEO Angle Swerve Configuration

This configuration uses **Kraken X60 (TalonFX)** motors for drive and **NEO (SparkMAX)** motors for steering/angle control.

## Motor Configuration

| Module | Drive Motor | Drive CAN ID | Angle Motor | Angle CAN ID | CANcoder ID |
|--------|-------------|--------------|-------------|--------------|-------------|
| Front Left | Kraken (talonfx) | 1 | NEO (sparkmax) | 5 | 9 |
| Front Right | Kraken (talonfx) | 2 | NEO (sparkmax) | 6 | 10 |
| Back Left | Kraken (talonfx) | 3 | NEO (sparkmax) | 7 | 11 |
| Back Right | Kraken (talonfx) | 4 | NEO (sparkmax) | 8 | 12 |

## Why Kraken + NEO Combo?

- **Kraken for Drive**: Higher torque and power for fast acceleration and high speeds
- **NEO for Angle**: Smooth, precise steering with excellent position control
- **Cost Effective**: NEOs are less expensive than Krakens while still providing excellent angle control

## Setup Instructions

### 1. Prepare the Robot
* Mount the robot on a cart with wheels not touching the sides
* Align all swerve modules with bevel gears pointing LEFT (robot front facing away from you)

![Robot Image](https://docs.yagsl.com/~gitbook/image?url=https%3A%2F%2F567506766-files.gitbook.io%2F%7E%2Ffiles%2Fv0%2Fb%2Fgitbook-x-prod.appspot.com%2Fo%2Fspaces%252F754c0Fpq8fBi6k4ByS1k%252Fuploads%252FP8kIHNspDzTMqBiHw1U1%252Fimage.png%3Falt%3Dmedia%26token%3D9923865e-47c7-4b1e-90a5-9da9610c0764&width=768&dpr=1&quality=100&sign=6a519e95&sv=2)

### 2. Configure CAN IDs

**For Kraken (Drive) motors:**
* Use Phoenix Tuner X to identify and set CAN IDs
* Verify drive motors: IDs 1-4

**For NEO (Angle) motors:**
* Use REV Hardware Client to identify and set CAN IDs
* Verify angle motors: IDs 5-8

**For CANcoders:**
* Use Phoenix Tuner X
* Verify encoders: IDs 9-12

### 3. Verify Motor Directions

**Drive Motors (Kraken/TalonFX):**
* Rotate drive wheel CCW (forward direction)
* In Phoenix Tuner, encoder value should INCREASE
* If not, set `"drive": true` in the `inverted` section

**Angle Motors (NEO/SparkMAX):**
* Rotate entire swerve module CCW (viewed from top)
* In REV Hardware Client, encoder value should INCREASE
* If not, set `"angle": true` in the `inverted` section

### 4. Set Absolute Encoder Offsets

1. Align all wheels straight forward using a straight edge
2. Zero out all `absoluteEncoderOffset` values in module JSON files
3. Deploy code and check Shuffleboard for `swerve/rawencodervalue`
4. Record values and enter them as `absoluteEncoderOffset` in each module file
5. Redeploy - values should now be near zero

### 5. Measure Module Locations

Measure from robot center to each module center (in inches):
* **Front/Back**: Positive = front, Negative = back
* **Left/Right**: Positive = left, Negative = right

Example for 24" x 24" robot:
* Front Left: `"front": 12.0, "left": 12.0`
* Front Right: `"front": 12.0, "left": -12.0`
* Back Left: `"front": -12.0, "left": 12.0`
* Back Right: `"front": -12.0, "left": -12.0`

## Physical Properties

Default values for MK4i modules with L2+ gearing:

| Property | Value | Notes |
|----------|-------|-------|
| Angle Gear Ratio | 21.43:1 | MK4i steering ratio (150/7) |
| Drive Gear Ratio | 6.75:1 | L2+ configuration |
| Wheel Diameter | 4.0 inches | Standard Colson/Billet wheel |
| Drive Current Limit | 60A | Kraken can handle higher current |
| Angle Current Limit | 40A | NEO limit |

## PID Tuning

### Drive Motors (Kraken)
Start with: `P: 0.1, I: 0, D: 0, F: 0`

### Angle Motors (NEO)
Start with: `P: 0.01, I: 0, D: 0, F: 0`

**Note:** NEO motors typically need lower P values than Krakens for angle control.

## Troubleshooting

### Angle motors oscillating
* Reduce angle P value (try 0.005)
* Add small D value (try 0.001)

### Drive motors sluggish
* Increase drive P value
* Check current limits

### Wheels not centering properly
* Verify absoluteEncoderOffset values
* Check CANcoder wiring
* Ensure angle motor direction is correct

## Files Overview

```
kraken-neo/
├── swervedrive.json          # IMU config, module list
├── controllerproperties.json # Heading PID
├── Readme.md                 # This file
└── modules/
    ├── frontleft.json        # FL module config
    ├── frontright.json       # FR module config
    ├── backleft.json         # BL module config
    ├── backright.json        # BR module config
    ├── physicalproperties.json # Gear ratios, limits
    └── pidfproperties.json   # Motor PID values
```
