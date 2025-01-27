package frc.robot.commands.algea.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OzzyGrabberSubsystem;

public class OzUp extends Command{
    OzzyGrabberSubsystem grabber;
    public OzUp(OzzyGrabberSubsystem subsystem) {
        grabber = subsystem;
        addRequirements(grabber);
    }
    @Override
    public void initialize() {
        // Initialization code here
    }
    @Override
    public void execute() {
        // Code to move the elevator
        if (grabber.up()) {
            end(false);
        }
    }
    @Override
    public void end(boolean interrupted) {
        // Code to stop the elevator
        
    }
    @Override
    public boolean isFinished() {
        // Condition to end the command
        return false;
    }
}
