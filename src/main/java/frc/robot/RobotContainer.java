package frc.robot;

import static edu.wpi.first.units.Units.*;

// WPILib Imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
    // Drivetrain: Using the constant from Tuner X generation
    // TO THIS (CORRECT):
public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Vision: Pass the drivetrain to it for pose updates
    private final VisionSubsystem m_vision = new VisionSubsystem(drivetrain);

    // Mechanisms
    private final TurretSubsystem m_turret = new TurretSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    // =========================================================================
    //  3. DASHBOARD & FIELD 2D
    // =========================================================================
    private final SendableChooser<Command> autoChooser;
    private final Field2d m_field = new Field2d(); // Visualizes robot on dashboard

    // =========================================================================
    //  4. SWERVE REQUESTS
    // =========================================================================
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
    .withDeadband(0.1).withRotationalDeadband(0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // =========================================================================
    //  CONSTRUCTOR
    // =========================================================================
    public RobotContainer() {
        configureBindings();

        // 1. Build the Auto Chooser
        // This automatically reads every ".auto" file you made in the PathPlanner GUI
        autoChooser = AutoBuilder.buildAutoChooser();

        // 2. Put widgets on the Dashboard
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Field", m_field);
    }

    // =========================================================================
    //  BINDINGS
    // =========================================================================
    private void configureBindings() {

        // --- DRIVER CONTROLS ---

        // Main Drive Command
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                     .withVelocityY(-m_driverController.getLeftX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                     .withRotationalRate(-m_driverController.getRightX() * (1.5 * Math.PI)) 
            )
        );

        // Vision Kill Switch (Back Button)
        m_driverController.back().onTrue(
            Commands.runOnce(() -> m_vision.disableVisionUpdates(), m_vision)
        );

        // Reset Gyro (Start Button)
        m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Auto-Align to Target (Hold 'A' button)
        m_driverController.a().whileTrue(new AlignToTarget(drivetrain, m_vision));

        // --- OPERATOR CONTROLS ---

        // 1. TURRET CONTROLS
        // Auto-home when the robot is enabled (safety check)
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
        m_operatorController.rightBumper().whileTrue(m_intake.runIntakeCommand(-0.8, -0.8));

        // 3. SHOOTER (Left Trigger) 
        m_operatorController.leftTrigger().whileTrue(m_shooter.runShooterCommand(-0.78, -.4, 0.78));
    }

    // =========================================================================
    //  METHODS
    // =========================================================================

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public CommandSwerveDrivetrain getDriveSubsystem() {
        return drivetrain; 
    }

    /**
     * Updates dashboard data. Call this from Robot.periodic()
     */
    public void updateDashboard() {
        // Update the Field2d widget with the robot's actual position
        m_field.setRobotPose(drivetrain.getState().Pose);
        
        // (Optional) Update vision debug values
        // SmartDashboard.putNumber("Turret Angle", m_turret.getCurrentAngle());
    }
}