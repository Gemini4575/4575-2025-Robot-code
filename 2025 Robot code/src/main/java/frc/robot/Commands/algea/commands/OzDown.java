package frc.robot.commands.algea.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OzzyGrabberSubsystem;

public class OzDown extends Command{
    OzzyGrabberSubsystem grabber;
    public OzDown(OzzyGrabberSubsystem subsystem) {
        grabber = subsystem;
        addRequirements(grabber);
    }
    @Override
    public void execute() {
        if(grabber.down()) {
            this.end(false);
        }
    }
}
