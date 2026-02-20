package frc.robot.subsystems;

// WPILib Imports
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Robot Imports
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    // =========================================================================
    //  1. CONFIGURATION
    // =========================================================================
    private final String kLimelightName = "limelight"; // Must match your Limelight's name in the web UI
    private final CommandSwerveDrivetrain m_drivetrain;
    
    // Kill Switch Flag
    private boolean m_isEnabled = true;

    // Trust Metrics (Standard Deviations)
    // Low numbers = High Trust. High numbers = Low Trust.
    // [x, y, theta] in meters and radians
    private static final double kTrustHigh = 0.1;   // Very trustworthy (close, multiple tags)
    private static final double kTrustMedium = 0.9; // Okay (mid-range, single tag)
    private static final double kTrustLow = 6.0;    // Do not trust (far away, high ambiguity)

    // =========================================================================
    //  2. CONSTRUCTOR
    // =========================================================================
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.m_drivetrain = drivetrain;

        // Turn off the LEDs by default so we don't blind everyone
        LimelightHelpers.setLEDMode_ForceOff(kLimelightName);
    }

    // =========================================================================
    //  3. PERIODIC LOOP (Runs every 20ms)
    // =========================================================================
    @Override
    public void periodic() {
        
        // 0. CHECK KILL SWITCH
        if (!m_isEnabled) return;

        // 1. SYNC GYRO (MegaTag 2 Fix)
        // We tell Limelight our precise rotation so it doesn't have to guess.
        // Note: Make sure CommandSwerveDrivetrain has a getHeadingDegrees() method!
        try {
            // If getHeadingDegrees() doesn't exist, use getPigeon2().getAngle()
           double robotHeading = m_drivetrain.getPigeon2().getYaw().getValueAsDouble(); 
            
            LimelightHelpers.SetRobotOrientation(
                kLimelightName, 
                robotHeading, 
                0, 0, 0, 0, 0 // We only care about Yaw (heading) for field positioning
            );
        } catch (Exception e) {
            // If drivetrain isn't ready, skip orientation update
        }

        // 2. CHECK SPIN RATE
        // Vision data lags behind reality. If we are spinning fast, that lag 
        // causes massive errors. We reject vision if spinning > 720 deg/sec.
        // (Assuming your drivetrain has this method, otherwise verify name)
        try {
             if (Math.abs(m_drivetrain.getState().Speeds.omegaRadiansPerSecond) > 4.0) { // ~230 deg/sec
                return; // Too fast! Ignore vision frame.
            }
        } catch (Exception e) {}

        // 3. GET POSE ESTIMATE
        // We use "botpose_orb_wpiblue" for MegaTag 2 (more stable)
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelightName);

        // 4. VALIDATION CHECKS
        // A. Did we see any tags?
        if (mt2.tagCount == 0) {
            return; // No tags seen, abort.
        }

        // B. Is the data fresh? (Don't use old data)
        if (mt2.timestampSeconds == 0) {
            return; 
        }

        // 5. CALCULATE CONFIDENCE (Standard Deviation)
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

        // 6. SEND TO DRIVETRAIN
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

    // =========================================================================
    //  4. HELPER METHODS (These fix the red lines in other files)
    // =========================================================================

    /**
     * Helper to get the horizontal offset from the Limelight.
     * @return tx value (degrees)
     */
    public double getTx() {
        return LimelightHelpers.getTX(kLimelightName);
    }
    
    // Capitalized version for compatibility
    public double getTX() {
        return LimelightHelpers.getTX(kLimelightName);
    }

    /**
     * Helper to check if the Limelight sees a tag.
     * @return true if tag is visible
     */
    public boolean hasTarget() {
        //return LimelightHelpers.getTV(kLimelightName); 
        return LimelightHelpers.getTV(kLimelightName); 
    }

    /**
     * Gets the vertical offset (ty) to the target.
     * Useful for calculating distance or shooter hood angle.
     */
    public double getTY() {
        return LimelightHelpers.getTY(kLimelightName);
    }  

    /**
     * Disables vision updates safely. 
     * Call this from a button binding if vision goes crazy during a match.
     */
    public void disableVisionUpdates() {
        m_isEnabled = false;
        System.out.println("WARNING: VISION DISABLED BY DRIVER");
    }

    /**
     * Gets the distance to the target.
     * JNL 02/11/2026 
     */
    public double getDistanceToTarget() {
        // 1. Get vertical angle offset from Limelight
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY(kLimelightName);

        // 2. Define your robot's constants (UPDATE THESE WITH REAL MEASUREMENTS!)
        final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 25.0; // Your camera's pitch
        final double LIMELIGHT_LENS_HEIGHT_INCHES = 20.0; // Your camera's height
        final double GOAL_HEIGHT_INCHES = 60.0; // The target's height (e.g. Reef or Source)

        // 3. Calculate the total angle to the goal
        double angleToGoalDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        // 4. Calculate the distance using trigonometry (tangent)
        // Distance = Height Difference / tan(Total Angle)
        double distanceFromLimelightToGoalInches = (GOAL_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }
}