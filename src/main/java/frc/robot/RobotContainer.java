package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Swerve Imports
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.generated.TunerConstants;

// Subsystem Imports
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    // 1. Controllers
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    // 2. Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final TurretSubsystem m_turret = new TurretSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    // 3. Swerve Requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(0.1).withRotationalDeadband(0.1) 
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        m_chooser.setDefaultOption("Do Nothing", Commands.none());
        SmartDashboard.putData("Auto Chooser", m_chooser);
    }

    private void configureBindings() {
        // --- DRIVER CONTROLS (Swerve) ---
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                     .withVelocityY(-m_driverController.getLeftX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                     .withRotationalRate(-m_driverController.getRightX() * (1.5 * Math.PI)) 
            )
        );

        // Reset Gyro (Start Button)
        m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        // --- OPERATOR CONTROLS ---
        // Turret Homing & Manual
        new Trigger(DriverStation::isEnabled).onTrue(m_turret.findHomeCommand());
        m_operatorController.a().onTrue(m_turret.findHomeCommand());
        m_operatorController.povRight().whileTrue(m_turret.rotateCommand(() -> 0.5));
        m_operatorController.povLeft().whileTrue(m_turret.rotateCommand(() -> -0.5));

        // Intake (Right Bumper)
        m_operatorController.rightBumper().whileTrue(m_intake.runIntakeCommand(0.7, 0.4));

        // Shooter (Left Trigger)
        m_operatorController.leftTrigger().whileTrue(m_shooter.runShooterCommand(-.7, -0.8, .7));
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
