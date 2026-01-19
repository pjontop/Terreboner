# Terreboner Robot Controls Documentation

## Overview
This document provides a complete reference for operating the Terreboner robot during competition and practice.

## Drive Controls

### Standard Controls
The robot uses **Split Arcade** control by default:
- **Left Joystick (Y-axis)**: Forward/Backward movement
- **Right Joystick (X-axis)**: Turning left/right

### Drive Settings
- **Brake Mode**: Coast (during operator control)
- **Curve Settings**: 3.5 (forward/reverse), 2.5 (turn) - optimized for precision descoring
- **Active Brake**: Disabled (kP = 0.0)

## Motor Controls

### Intake System
- **L1 (Left Bumper)**: Run intake motor forward (127 speed)
- **L2 (Left Trigger)**: Run outake motor forward (127 speed)
  - Motors stop automatically when buttons are released

### Lever Mechanism
- **R2 (Right Trigger)**: 
  - **While Held**: Lever moves forward (127 speed)
  - **On Release**: Automatically reverses briefly (~150ms) then stops
  - This auto-reverse feature helps with reliable ring manipulation

### Manual Outake Override
- **D-Pad LEFT**: Reverse outake motor
- **D-Pad RIGHT**: Forward outake motor
  - Use these for fine-tuned manual control when needed

## Pneumatic Controls

All pneumatic controls use **toggle mode** (press to change state, press again to return):

- **R1 (Right Bumper)**: Toggle descore wing (ADI port H)
- **D-Pad UP**: Toggle match load mechanism (ADI port B)
- **D-Pad DOWN**: Toggle forebar (ADI port A)

## Developer/Practice Features

### PID Tuner (Practice Only)
- **X Button**: Toggle PID tuning mode
  - Only available when NOT connected to competition switch
  - Used for calibrating autonomous movement

### Test Autonomous (Practice Only)
- **B + D-Pad DOWN**: Run the selected autonomous routine
  - Only available when NOT connected to competition switch
  - Useful for testing autonomous programs during practice

### Curve Adjustment (If Enabled)
If `opcontrol_curve_buttons_toggle(true)` is enabled:
- Use controller buttons to adjust the drive curve during operation
- Allows real-time tuning of drive sensitivity

## Robot Hardware Configuration

### Drive Motors
- **Left Side**: Ports 11, 6, 12 (reversed)
- **Right Side**: Ports 19, 10, 9
- **Wheel Size**: 4.125" diameter
- **Wheel RPM**: 600

### Mechanism Motors
- **Intake**: Port 20 (reversed)
- **Outake**: Port 20
- **Lever**: Port 14 (reversed)

### Pneumatics (ADI Ports)
- **Match Load**: Port B
- **Forebar**: Port A
- **Descore Wing**: Port H

### Sensors
- **IMU (Gyro)**: Port 13
- **Horizontal Tracking Wheel**: Port 8 (2.125" diameter, -1.57 offset)
- **Vertical Tracking Wheel**: Port 18 (2.125" diameter, 0 offset)

## Tips & Best Practices

1. **Driving**: The robot uses a precision curve (3.5/2.5) optimized for descoring - movements are more precise but may feel slower
2. **Lever Control**: Take advantage of the auto-reverse feature by simply releasing R2 after scoring
3. **Pneumatics**: All pneumatics are toggle-based, so be mindful of current state
4. **Practice Mode**: Use X to access PID tuning and B+DOWN to test autonomous routines during practice

## Autonomous Routines

The following autonomous routines are available (selected via brain screen):
1. **Skills**: Full field skills autonomous
2. **AWP**: Solo AWP routine (start at right)
3. Various test routines for development

## Competition Notes

- PID Tuner and Test Autonomous features are automatically disabled when connected to competition switch
- Robot will rumble controller on initialization:
  - Short buzz (`.`): IMU calibrated successfully
  - Long buzz (`---`): IMU calibration failed

## Troubleshooting

**Robot not responding to controls:**
- Check that opcontrol mode is active (not autonomous or disabled)
- Verify controller is properly paired

**Drive feels too slow/fast:**
- Adjust curve settings in `main.cpp` line 27
- Current settings: `chassis.opcontrol_curve_default_set(3.5, 2.5)`

**Lever not auto-reversing:**
- This is normal behavior - it requires you to have held R2 first
- Auto-reverse duration is ~150ms (15 cycles × 10ms delay)
