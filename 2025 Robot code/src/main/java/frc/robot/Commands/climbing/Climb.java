package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.NickClimbingSubsystem;

public class Climb extends Command{
    
    private NickClimbingSubsystem climbing;

    public Climb(NickClimbingSubsystem subsystem) {
        // Initialization code here
        climbing = subsystem;
    }

    @Override
    public void initialize() {
        // Initialization code here
    }

    @Override
    public void execute() {
        // Code to move the elevator
        if(climbing.Climb()) {
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Code to stop the elevator
    }
    
}
