package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.VisionSubsystem;

public class Blue_Side_G extends Command{
      private final VisionSubsystem m_subsystem;
      private TelopSwerve m_telopSwerve;
      private DriveTrain s_Swerve;
      private int Side;
    
      /**
       * Creates a new ExampleCommand.
       *
       * @param subsystem The subsystem used by this command.
       */
      public Blue_Side_G(VisionSubsystem subsystem, DriveTrain s_Swerve, int Side) {
        m_subsystem = subsystem;
        this.s_Swerve = s_Swerve;
        this.Side = Side;
        
        addRequirements(subsystem, s_Swerve);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        m_telopSwerve = new TelopSwerve(s_Swerve,
         () -> m_subsystem.processImage(Side).vxMetersPerSecond,
         () -> m_subsystem.processImage(Side).vyMetersPerSecond,
         () -> m_subsystem.processImage(Side).omegaRadiansPerSecond
         );
        
      }
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
    }
    

