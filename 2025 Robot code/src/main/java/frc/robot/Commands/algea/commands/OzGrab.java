package frc.robot.commands.algea.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OzzyGrabberSubsystem;

public class OzGrab extends Command{
    OzzyGrabberSubsystem grabber;
    public OzGrab(OzzyGrabberSubsystem subsystem) {
        this.grabber = subsystem;
        addRequirements(grabber);
    }

    @Override
    public void execute() {
        if(grabber.Grab()) {
            this.end(false);
        }
    }
}
