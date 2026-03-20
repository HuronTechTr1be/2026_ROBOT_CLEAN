# Tuning Guide

## Pivot Arm Positions
- `src/main/java/frc/robot/subsystems/PivotSubsystem.java` **line 25-26**
  - `kPositionUp = 0.0` (degrees) — change this to the desired "up" angle
  - `kPositionDown = 90.0` (degrees) — change this to the desired "down" angle

## Current Limits (Amps)
- `src/main/java/frc/robot/subsystems/PivotSubsystem.java` **line 40**
  - Pivot NEO — currently 25A
- `src/main/java/frc/robot/subsystems/IntakeBarsSubsystem.java` **line 18**
  - Intake bars Kraken — currently 40A

## Roller Bar Speeds
- `src/main/java/frc/robot/subsystems/IntakeBarsSubsystem.java` **line 35**
  - Fast speed — currently 0.9
- `src/main/java/frc/robot/subsystems/IntakeBarsSubsystem.java` **line 39**
  - Slow speed — currently 0.4
