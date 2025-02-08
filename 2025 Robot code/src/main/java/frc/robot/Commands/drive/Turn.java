package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.service.TurnService;
import frc.robot.subsystems.drive.DriveTrain;

public class Turn extends Command {

    private final TurnService turnService;

    public Turn(DriveTrain driveTrain) {
        turnService = new TurnService(driveTrain);
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        turnService.startTurning(90, Math.PI/90.0);
    }

    @Override
    public void execute() {
        turnService.keepTurning();
    }

    @Override
    public boolean isFinished() {
        return !turnService.keepTurning();
    }

}
