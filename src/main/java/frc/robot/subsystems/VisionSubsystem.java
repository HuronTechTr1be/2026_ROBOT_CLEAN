package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants; // Import your swerve constants

public class VisionSubsystem extends SubsystemBase {

    // 1. CONFIGURATION
    private final String kLimelightName = "limelight"; // Must match your Limelight's name in the web UI
    private final CommandSwerveDrivetrain m_drivetrain;

    // Trust Metrics (Standard Deviations)
    // Low numbers = High Trust. High numbers = Low Trust.
    // [x, y, theta] in meters and radians
    private static final double kTrustHigh = 0.1;  // Very trustworthy (close, multiple tags)
    private static final double kTrustMedium = 0.9; // Okay (mid-range, single tag)
    private static final double kTrustLow = 6.0;   // Do not trust (far away, high ambiguity)

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.m_drivetrain = drivetrain;

        // Turn off the LEDs by default so we don't blind everyone
        LimelightHelpers.setLEDMode_ForceOff(kLimelightName);
    }

    /**
     * This method runs automatically every 20ms (loop).
     * It pulls data from the camera and feeds it to the Drivetrain.
     */
    @Override
    public void periodic() {
        // 1. Get the latest Pose Estimate from LimelightHelpers
        // We use "botpose_orb_wpiblue" for MegaTag 2 (more stable) in Blue Alliance coordinates
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelightName);

        // 2. VALIDATION CHECKS
        
        // A. Did we see any tags?
        if (mt2.tagCount == 0) {
            return; // No tags seen, abort.
        }

        // B. Is the data fresh? (Don't use old data)
        if (mt2.timestampSeconds == 0) {
            return; 
        }

        // C. Is the robot spinning too fast? 
        // Vision lags behind reality. If we are spinning fast, that lag causes massive errors.
        // We reject vision updates if angular velocity is > 720 deg/sec (2 rotations/sec)
        // (You can tune this threshold)
        /* // Requires access to gyro rate. Uncomment if your Drivetrain exposes getGyroRate()
        if (Math.abs(m_drivetrain.getPigeon2().getRate()) > 720) {
            return;
        }
        */

        // D. Calculate Confidence (Standard Deviation)
        // If we are far away or have few tags, we trust the vision less.
        double xyStds = kTrustMedium;
        double degStds = kTrustMedium;

        // If we have multiple tags, trust it more!
        if (mt2.tagCount >= 2) {
            xyStds = kTrustHigh;
            degStds = kTrustHigh;
        } 
        // If the average tag distance is > 4 meters, trust it less!
        else if (mt2.avgTagDist > 4.0) {
            xyStds = kTrustLow;
            degStds = kTrustLow;
        }

        // 3. SEND TO DRIVETRAIN
        // We pass the Pose, the Timestamp (latency compensated), and our calculated Trust
        try {
            m_drivetrain.addVisionMeasurement(
                mt2.pose, 
                mt2.timestampSeconds, 
                VecBuilder.fill(xyStds, xyStds, degStds)
            );
            
            // Debugging
            SmartDashboard.putString("Vision/Pose", mt2.pose.toString());
            SmartDashboard.putNumber("Vision/TagCount", mt2.tagCount);
        } catch (Exception e) {
            // Sometimes the matrix math fails if the pose is NaN
            System.out.println("Vision Update Failed: " + e.getMessage());
        }
    }

    // --- Helper Methods for Commands ---

    /**
     * Checks if the Limelight currently sees a valid target (any tag or object).
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(kLimelightName);
    }

    /**
     * Gets the horizontal offset (tx) to the target.
     * Useful for aiming turrets or the whole robot.
     */
    public double getTX() {
        return LimelightHelpers.getTX(kLimelightName);
    }

    /**
     * Gets the vertical offset (ty) to the target.
     * Useful for calculating distance or shooter hood angle.
     */
    public double getTY() {
        return LimelightHelpers.getTY(kLimelightName);
    }
}