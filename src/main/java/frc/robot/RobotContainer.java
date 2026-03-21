package frc.robot;

import static edu.wpi.first.units.Units.*;

// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// PathPlanner Import
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

// Swerve Imports
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.generated.TunerConstants;

// Subsystem Imports
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.TurretSubsystem;
//import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IntakeBarsSubsystem;


// Command Imports (commented out until subsystems are re-enabled)
//import frc.robot.Commands.AlignToTarget;
//import frc.robot.Commands.AutoTurretAlign;
//import frc.robot.Commands.TurretAutoTrack;

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
    private final PivotSubsystem m_pivot = new PivotSubsystem();
    private final IntakeBarsSubsystem m_intakeBars = new IntakeBarsSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    // private final VisionSubsystem m_vision = new VisionSubsystem(drivetrain);
    // private final TurretSubsystem m_turret = new TurretSubsystem();
    // private final ShooterSubsystem m_shooter = new ShooterSubsystem();
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

        // =========================================================================
        //  PATHPLANNER NAMED COMMANDS
        // =========================================================================
        // These MUST exactly match the spelling in your PathPlanner App!
        
       // NamedCommands.registerCommand("AlignTurretToTag", new AutoTurretAlign(m_turret, m_vision));

        // "Leaving To Collect" - Placeholder print command so PathPlanner doesn't crash
        NamedCommands.registerCommand("print message", Commands.print("print message...")); 

        // "Collection" - Runs the intake for 2 seconds, then finishes so the next path can start
      //  NamedCommands.registerCommand("intake", m_intake.runIntakeCommand(-1.0, -1.0).withTimeout(5.0)); 

        // "Return" - Runs the shooter for 2 seconds to score, then finishes
      //  NamedCommands.registerCommand("shoot", m_shooter.runShooterRPMCommand(-2450, -.25, 2450).withTimeout(5.0));


        // =========================================================================
        //  BUILD AUTO CHOOSER
        // =========================================================================
        // This automatically reads every ".auto" file you made in the PathPlanner GUI
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Field", m_field);
    }

    // =========================================================================
    //  BINDINGS
    // =========================================================================
    private void configureBindings() {

        // --- DRIVER CONTROLS ---

        // Main Drive Command with Slow Mode on Right Trigger
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                //If Right Trigger is pressed more than halfway (0.5), multiplier is 0.4
                double multiplier = m_driverController.getRightTriggerAxis() > 0.5 ? 0.4 : 1.0; 

                return drive
                    .withVelocityX(-m_driverController.getLeftY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * multiplier)
                    .withVelocityY(-m_driverController.getLeftX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * multiplier)
                    .withRotationalRate(-m_driverController.getRightX() * (1.5 * Math.PI) * multiplier);
            })
        );

        // Vision Kill Switch (Back Button) — disabled until VisionSubsystem is re-enabled
        // m_driverController.back().onTrue(
        //     Commands.runOnce(() -> m_vision.disableVisionUpdates(), m_vision)
        // );

        // Reset Gyro (Start Button)
        m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Auto-Align to Target (Hold 'A' button)
      //  m_operatorController.a().whileTrue(new AlignToTarget(drivetrain, m_vision));

        // --- OPERATOR CONTROLS ---
        
        // 1. TURRET CONTROLS
       // new Trigger(DriverStation::isEnabled).onTrue(m_turret.findHomeCommand());
      //  m_operatorController.a().onTrue(m_turret.findHomeCommand());
        //m_operatorController.x().whileTrue(new TurretAutoTrack(m_turret, m_vision));

     //   m_operatorController.povRight().onTrue(m_turret.goToAngleCommand(45));
      //  m_operatorController.povLeft().onTrue(m_turret.goToAngleCommand(-45));
    //    m_operatorController.povUp().onTrue(m_turret.goToAngleCommand(0));

        // 4. INTAKE BARS / ROLLER BARS (Driver D-Pad Up/Down)
        m_driverController.povUp().whileTrue(Commands.run(() -> m_intakeBars.setSpeed(0.3), m_intakeBars).finallyDo(() -> m_intakeBars.stop()));
        m_driverController.povDown().whileTrue(Commands.run(() -> m_intakeBars.setSpeed(-0.3), m_intakeBars).finallyDo(() -> m_intakeBars.stop()));

        // 5. PIVOT MECHANISM (Driver LB/RB)
        m_operatorController.leftTrigger().whileTrue(m_pivot.lowerCommand());
        m_operatorController.rightTrigger().whileTrue(m_pivot.raiseCommand());
        //HERE!!

        // 6. PIVOT POSITION (Operator A = down, Y = up) — hold to move, release to hold
        //m_operatorController.a().whileTrue(m_pivot.goDownCommand());
        //m_operatorController.y().whileTrue(m_pivot.goUpCommand());

        // 2. INTAKE BARS SPEED (Operator B = fast, X = slow)
        m_operatorController.b().whileTrue(m_intakeBars.runFastCommand());
        m_operatorController.x().whileTrue(m_intakeBars.runSlowCommand());

        // 3. INTAKE (Right Bumper)
        m_operatorController.rightBumper().whileTrue(m_intake.runIntakeCommand(-0.1, -0.1));

        // Reverse Intake (Left Bumper) - Outtakes the object
        m_operatorController.leftBumper().whileTrue(m_intake.runIntakeCommand(0.5, 0.5));  

        // 3. SHOOTER (Left Trigger) 
       // m_operatorController.leftTrigger().whileTrue(m_shooter.runShooterRPMCommand(-2500, -.25, 2500));
      //  m_operatorController.leftTrigger().whileTrue(m_intake.runIntakeCommand(-.9, -.9));
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

    // public void updateDashboard() {
    //     // Update the Field2d widget with the robot's actual position
    //     //m_field.setRobotPose(drivetrain.getState().Pose);
    // }
}