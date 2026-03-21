# Tuning Guide

## Pivot Arm Positions
- `src/main/java/frc/robot/subsystems/PivotSubsystem.java` **line 25-26**
  - `kPositionUp = 0.0` (degrees) — change this to the desired "up" angle
  - `kPositionDown = 75` (degrees) — change this to the desired "down" angle

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

---

## TODOs — Still Needs Tuning

| What | File | Line | Current Value |
|------|------|------|---------------|
| Pivot up/down positions | `PivotSubsystem.java` | lines 25-26 | Up = 0.0, Down = 75 deg |
| Pivot PID gains | `PivotSubsystem.java` | 29-31 | kP = 0.1, kI = 0.0, kD = 0.0 |
| Intake bars fast/slow speeds | `IntakeBarsSubsystem.java` | 35, 39 | Fast = 0.9, Slow = 0.4 |

---

## Driver Controller (Port 0)

All driver bindings are in `src/main/java/frc/robot/RobotContainer.java` inside `configureBindings()`, starting at **line 100**.

| Button | Action | Hold/Press | Line | Notes |
|--------|--------|------------|------|-------|
| Left Stick | Drive X/Y | Hold | 105-115 | Forward/backward and strafe |
| Right Stick X | Rotation | Hold | 113 | Spin the robot |
| Right Trigger | Slow Mode | Hold | 108 | Cuts speed to 40% when held past halfway |
| Start | Reset Gyro | Press | 123 | Re-zeros field-centric heading |
| D-Pad Up | Intake Bars Forward | Hold | 140 | Runs bars at 0.3 speed |
| D-Pad Down | Intake Bars Reverse | Hold | 141 | Runs bars at -0.3 speed |

## Operator Controller (Port 1)

All operator bindings are in `src/main/java/frc/robot/RobotContainer.java` inside `configureBindings()`, starting at **line 128**.

| Button | Action | Hold/Press | Line | Notes |
|--------|--------|------------|------|-------|
| A | Pivot Down | Hold | 149 | Goes to kPositionDown (75 deg) |
| Y | Pivot Up | Hold | 150 | Goes to kPositionUp (0 deg) |
| B | Intake Bars Fast | Hold | 153 | Runs bars at 0.5 speed |
| X | Intake Bars Slow | Hold | 154 | Runs bars at 0.4 speed |
| RB (Right Bumper) | Intake In | Hold | 157 | Runs intake at -0.1 speed |
| LB (Left Bumper) | Intake Out (Outtake) | Hold | 160 | Reverses intake rollers |
| Left Trigger | Pivot Lower | Hold | 144 | Manual lower, stops on release |
| Right Trigger | Pivot Raise | Hold | 145 | Manual raise, stops on release |

