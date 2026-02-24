package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurretAutoTrack extends Command {
    private final TurretSubsystem m_turret;
    private final VisionSubsystem m_vision;

    // Tuning Constant (kP):
    // 0.03 means: If target is 10 degrees off, run motor at 30% speed (10 * 0.03 = 0.3)
    // If it oscillates (shakes left/right), make this number SMALLER (e.g., 0.01)
    // If it's too slow, make this number BIGGER (e.g., 0.05)
    private final double kP = 0.03; 

    public TurretAutoTrack(TurretSubsystem turret, VisionSubsystem vision) {
        m_turret = turret;
        m_vision = vision;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        int[] targetIds = {8, 10, 11, 24, 26, 27};
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", targetIds);
        // 1. Check if Limelight sees a tag
        if (m_vision.hasTarget() && m_vision.getTV() ){
            // 2. Get the error (tx)
          //  if (LimelightHelpers.getTV("limelight")) {
            //double tagId = LimelightHelpers.getFiducialID("limelight");
        //}
            double tx = m_vision.getTX();
            double tz = m_vision.getDistanceToTarget();
            System.out.println("Target Aquired " + tx);        //JNL 02/11/2026 
            System.out.println("Distance " + tz);        //JNL 02/11/2026 
            // 3. Calculate speed
            // NOTE: You might need to change this to "-tx" if it turns the wrong way!
            double speed = tx * kP; 
            // calc movement
            double m = -1; // gain offset 
            double b = 0; // 0 offset
            double txa = m * tx + b; 
            // 4. Move the turret
           // m_turret.runManual(speed);
           m_turret.setAngle(txa); 
        }
            
        //} else {
        //     //No target? Stop moving.
        //    m_turret.runManual(0);
        }
    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the command finishes
        m_turret.runManual(0);
    }
    }

    
    
    

