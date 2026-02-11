package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// PathPlanner Import
import com.pathplanner.lib.auto.AutoBuilder;

// Swerve Imports
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.generated.TunerConstants;

// Subsystem Imports
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// Command Imports
import frc.robot.Commands.AlignToTarget;
import frc.robot.Commands.TurretAutoTrack;

public class RobotContainer {

    // =========================================================================
    //  1. CONTROLLERS
    // =========================================================================
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    // =========================================================================
//  2. SUBSYSTEMS
// =========================================================================
// Use the Drivetrain that Tuner X generated for you
// Correct way for your generated code: call the method with parentheses ()
private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

// Pass that drivetrain into Vision so they can talk
private final VisionSubsystem m_vision = new VisionSubsystem(drivetrain); 

private final TurretSubsystem m_turret = new TurretSubsystem();
private final IntakeSubsystem m_intake = new IntakeSubsystem();
private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    // =========================================================================
    //  3. SWERVE REQUESTS
    // =========================================================================
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(0.1).withRotationalDeadband(0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // =========================================================================
    //  4. AUTO CHOOSER
    // =========================================================================
    private final SendableChooser<Command> m_chooser;

    // =========================================================================
    //  CONSTRUCTOR
    // =========================================================================
    public RobotContainer() {
        // Configure the button bindings
        configureBindings();

        // Build the Auto Chooser (Must happen after Drivetrain is initialized)
        m_chooser = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putData("Auto Chooser", m_chooser);
    }

    // =========================================================================
    //  BINDINGS
    // =========================================================================
    private void configureBindings() {
        
        // --- DRIVER CONTROLS (Swerve) ---
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(m_driverController.getLeftY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                     .withVelocityY(m_driverController.getLeftX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                     .withRotationalRate(-m_driverController.getRightX() * (1.5 * Math.PI)) 
            )
        );

        // Reset Gyro (Start Button)
       // NEW (2026)
m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // Auto-Align to Target (Hold 'A' button)
        m_driverController.a().whileTrue(new AlignToTarget(drivetrain, m_vision));

        // --- OPERATOR CONTROLS ---

        // 1. TURRET CONTROLS
        // Auto-home when the robot is enabled
        new Trigger(DriverStation::isEnabled).onTrue(m_turret.findHomeCommand());
        
        // Manual Homing Button (Button A)
        m_operatorController.a().onTrue(m_turret.findHomeCommand());

        // Track AprilTag with Turret (Hold 'X' button)
        m_operatorController.x().whileTrue(new TurretAutoTrack(m_turret, m_vision));

        // D-Pad Positioning
        m_operatorController.povRight().onTrue(m_turret.goToAngleCommand(45));
        m_operatorController.povLeft().onTrue(m_turret.goToAngleCommand(-45));
        m_operatorController.povUp().onTrue(m_turret.goToAngleCommand(0));

        // 2. INTAKE (Right Bumper)
        m_operatorController.rightBumper().whileTrue(m_intake.runIntakeCommand(0.7, 0.4));

        // 3. SHOOTER (Left Trigger) 
        // Args: LeftMotor, LowerMotor, RightMotor
        m_operatorController.leftTrigger().whileTrue(m_shooter.runShooterCommand(-0.7, -1.0, 0.7));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public CommandSwerveDrivetrain getDriveSubsystem() {
        return drivetrain; 
    }

    public VisionSubsystem getVisionSubsystem() {
        return m_vision; 
    }
}