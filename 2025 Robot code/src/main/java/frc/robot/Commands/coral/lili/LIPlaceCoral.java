package frc.robot.commands.coral.lili;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.LiliCoralSubystem;
import frc.robot.Subsystems.drive.DriveTrain;
import frc.robot.commands.drive.DriveXMeters;

public class LIPlaceCoral extends SequentialCommandGroup{
    public LIPlaceCoral(LiliCoralSubystem c, DriveTrain s) {
        addCommands(
            new EXODropGate(c),
            new WaitCommand(1),
            //TODO maybe dont automate this
            new DriveXMeters(s, 0.051),
            new LIIntakeCoral(c)
        );
    }
}
