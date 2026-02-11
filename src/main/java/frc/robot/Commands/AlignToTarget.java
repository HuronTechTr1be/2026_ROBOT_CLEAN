package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain; // Imports your specific drivetrain
import frc.robot.subsystems.VisionSubsystem;       // Imports your vision subsystem
import com.ctre.phoenix6.swerve.SwerveRequest;     // REQUIRED for CTRE Swerve

public class AlignToTarget extends Command {
    private final CommandSwerveDrivetrain driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final PIDController turnController;

    // Create a request object to tell CTRE how to drive
    // RobotCentric is best for spinning in place
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    public AlignToTarget(CommandSwerveDrivetrain drive, VisionSubsystem vision) {
        this.driveSubsystem = drive;
        this.visionSubsystem = vision;
        addRequirements(drive);

        // Tuning: kP = 0.04. 
        // If it shakes, lower to 0.01. If it's too slow, raise to 0.06.
        turnController = new PIDController(0.04, 0.0, 0.001); 
        turnController.setTolerance(1.0); // Stop when within 1 degree
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTarget()) {
            double tx = visionSubsystem.getTX();
            
            // 1. Calculate the PID output (-1.0 to 1.0)
            double pidOutput = turnController.calculate(tx, 0); 
            
            // 2. Convert to Radians Per Second for CTRE
            // We multiply by ~5.0 to give it enough speed (Max spin is roughly 5-10 rad/s)
            double rotationalRate = pidOutput * 5.0; 

            // 3. Apply to drivetrain using setControl (The CTRE way)
            driveSubsystem.setControl(
                driveRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(rotationalRate)
            );

        } else {
            // Safety: Stop if no target
            driveSubsystem.setControl(
                driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Safety stop when command finishes
        driveSubsystem.setControl(
            driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}