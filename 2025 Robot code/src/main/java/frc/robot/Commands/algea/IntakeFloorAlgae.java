package frc.robot.commands.algea;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.OzzyGrabberSubsystem;
import frc.robot.commands.algea.commands.OzGrab;
import frc.robot.commands.algea.commands.OzIntake;
import frc.robot.commands.algea.commands.OzUp;

public class IntakeFloorAlgae extends SequentialCommandGroup{
    public IntakeFloorAlgae(OzzyGrabberSubsystem grabber){
        addCommands(
            new OzGrab(grabber),
            new OzIntake(grabber),
            new OzUp(grabber)
        );
    }
}
