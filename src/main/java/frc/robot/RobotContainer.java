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
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem m_vision = new VisionSubsystem(drivetrain);
    private final TurretSubsystem m_turret = new TurretSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    // =========================================================================
    //  3. DASHBOARD & FIELD 2D
    // =========================================================================
    private final SendableChooser<Command> autoChooser;
    private final Field2d m_field = new Field2d(); 

    // =========================================================================
    //  4. SWERVE REQUESTS
    // =========================================================================
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
    .withDeadband(0.1).withRotationalDeadband(0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Field", m_field);
    }

    private void configureBindings() {
        // --- DRIVER CONTROLS ---
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                     .withVelocityY(-m_driverController.getLeftX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                     .withRotationalRate(-m_driverController.getRightX() * (1.5 * Math.PI)) 
            )
        );

        m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        m_driverController.a().whileTrue(new AlignToTarget(drivetrain, m_vision));

        // --- OPERATOR CONTROLS ---
        
        // Turret Controls
        new Trigger(DriverStation::isEnabled).onTrue(m_turret.findHomeCommand());
        m_operatorController.a().onTrue(m_turret.findHomeCommand());
        m_operatorController.x().whileTrue(new TurretAutoTrack(m_turret, m_vision));

        // D-Pad Turret Positioning
        m_operatorController.povRight().onTrue(m_turret.goToAngleCommand(-45));
        m_operatorController.povLeft().onTrue(m_turret.goToAngleCommand(45));
        m_operatorController.povUp().onTrue(m_turret.goToAngleCommand(0));

        // Intake (Right Bumper)
        m_operatorController.rightBumper().whileTrue(m_intake.runIntakeCommand(-0.8, -0.8));

        // SHOOTER + INTAKE (Left Trigger)
        // This runs both at once!
        m_operatorController.leftTrigger().whileTrue(
            m_shooter.runShooterCommand(-0.78, -.4, 0.78)
            .alongWith(m_intake.runIntakeCommand(-0.8, -0.8))
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void updateDashboard() {
        m_field.setRobotPose(drivetrain.getState().Pose);
    }
}