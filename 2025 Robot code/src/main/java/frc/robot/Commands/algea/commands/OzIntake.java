package frc.robot.commands.algea.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OzzyGrabberSubsystem;

public class OzIntake extends Command{
    OzzyGrabberSubsystem grabber;
    public OzIntake(OzzyGrabberSubsystem subsystem) {
        this.grabber = subsystem;
        addRequirements(grabber);
    }

    @Override
    public void execute() {
        if(grabber.intake()){
            this.end(false);
        }
    }
}
