package frc.robot.commands.algea;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.OzzyGrabberSubsystem;
import frc.robot.commands.algea.commands.OzDown;

public class IntakeLolipopAlgae extends SequentialCommandGroup{
    public IntakeLolipopAlgae(OzzyGrabberSubsystem grabber, BooleanSupplier wait) {
        addCommands(
            new OzDown(grabber),
            new Oz
        );
    }
}
