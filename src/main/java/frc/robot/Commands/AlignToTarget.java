/*package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class AlignToTarget extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    
    // INCREASED kP: 0.15 means "For 10 degrees error, spin at 1.5 radians/sec"
    private final PIDController turnController = new PIDController(0.15, 0, 0);

    // We use a RobotCentric drive request for aligning (simpler math)
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AlignToTarget(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        
        // CRITICAL: This line tells the scheduler "Pause the Joystick Drive while I run"
        addRequirements(drivetrain);
        
        // Stop when we are within 1.5 degrees of the target
        turnController.setTolerance(1.5);
    }

    @Override
    public void execute() {
        double rotationSpeed = 0;
        boolean hasTarget = vision.hasTarget();
        double tagID = LimelightHelpers.getFiducialID(getName());
        double tx = vision.getTx();

        // 1. Check if Limelight sees a tag
        if (hasTarget) {
            // 2. Calculate Speed (Note: We use negative tx because +tx means target is to the right)
            rotationSpeed = turnController.calculate(-tx, 0); 
            System.out.println("hastarget... Tx:  HasTarget: ");
            // 3. MINIMUM SPEED CHECK (Friction fighter)
            // If the speed is tiny (but not zero), boost it so the robot actually moves
            if (Math.abs(rotationSpeed) < 0.1 && Math.abs(rotationSpeed) > 0.01) {
                rotationSpeed = Math.copySign(0.1, rotationSpeed);
            }
        } else {
           rotationSpeed = 0;
        }

        // 4. DEBUG PRINT (Check your "Riolog" in VS Code)
        // This will tell you if the command is actually running
        System.out.println("Aligning... Tx: " + tx + " | Speed: " + rotationSpeed + " | HasTarget: " + hasTarget);

        // 5. Send command to Swerve
        // X=0, Y=0, Rotation=Calculated
      //  drivetrain.setControl(driveRequest
        //    .withVelocityX(0)
          //  .withVelocityY(0)
            //.withRotationalRate(rotationSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Align Command Ended.");
        // Stop motors when button is released
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}
    */