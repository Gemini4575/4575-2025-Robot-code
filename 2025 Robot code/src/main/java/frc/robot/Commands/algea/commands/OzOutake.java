package frc.robot.commands.algea.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OzzyGrabberSubsystem;

public class OzOutake extends Command{
    OzzyGrabberSubsystem grabber;
    public OzOutake(OzzyGrabberSubsystem subsystem) {
        this.grabber = subsystem;
        addRequirements(grabber);
    }

    @Override
    public void execute() {
        if(grabber.outake()) {
            this.end(false);
        }
    }
}
