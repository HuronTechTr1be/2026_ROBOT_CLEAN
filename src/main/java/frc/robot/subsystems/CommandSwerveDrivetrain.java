package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

// --- PATHPLANNER 2025 IMPORTS ---
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards; 

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// --- DASHBOARD IMPORTS ---
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    // =========================================================================
    //  VARIABLES
    // =========================================================================
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    // Field2d Widget
    private final Field2d m_field = new Field2d();

    // =========================================================================
    //  SYSID VARIABLES
    // =========================================================================
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null,
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(7), null,
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null,
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            }, null, this));

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    // =========================================================================
    //  CONSTRUCTORS
    // =========================================================================
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(drivetrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    // =========================================================================
    //  PATHPLANNER SETUP
    // =========================================================================
    public void configurePathPlanner() {
        
        // Push the Field to the dashboard
        SmartDashboard.putData("Field", m_field);

        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose, 
                this::resetPose, // Now correctly linked to the method below!
                this::getRobotRelativeSpeeds, 
                (ChassisSpeeds speeds, DriveFeedforwards feedforwards) -> {
                    this.setControl(autoRequest.withSpeeds(speeds));
                },
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID
                    new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
                ),
                config, 
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
    }

    // =========================================================================
    //  HELPER METHODS
    // =========================================================================
    
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getState().Speeds;
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    // Resets the robot's pose on the field (fixes the disabling issue!)


    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // Continually update the dashboard widget with the robot's pose
        m_field.setRobotPose(this.getPose());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // =========================================================================
    //  VISION & GYRO METHODS
    // =========================================================================

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * Gets the current robot rotation in degrees.
     * Useful for syncing with Limelight.
     */
    public double getHeadingDegrees() {
        return this.getPigeon2().getYaw().getValueAsDouble();
    }

    /**
     * Gets the current rotation speed in degrees per second.
     * Useful for rejecting vision updates when spinning fast.
     */
    public double getRotationalRate() {
        return this.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
    }

}