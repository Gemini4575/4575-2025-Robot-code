package frc.robot.commands.auto;

import static frc.robot.datamodel.MotionDirective.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.StartMotionSequence;
import frc.robot.commands.coral.lili.LIPlaceCoral;
import frc.robot.service.MotionService;
import frc.robot.subsystems.LiliCoralSubystem;

public class DriveAndDropToOne extends SequentialCommandGroup{
    public DriveAndDropToOne(MotionService s, LiliCoralSubystem c) {
        addCommands(
            new StartMotionSequence(s, drive(Units.inchesToMeters(84))),
            new LIPlaceCoral(c)
        );
    }
}
