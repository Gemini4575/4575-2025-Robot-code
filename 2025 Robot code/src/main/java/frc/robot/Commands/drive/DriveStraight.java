package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.service.DriveService;
import frc.robot.subsystems.drive.DriveTrain;

public class DriveStraight extends Command {

    private final DriveService driveService;

    public DriveStraight(DriveTrain driveTrain) {
        driveService = new DriveService(driveTrain);
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveService.startDriving(1, 1, 0.005);
    }

    @Override
    public void execute() {
        driveService.keepDriving();
    }

    @Override
    public boolean isFinished() {
        return !driveService.keepDriving();
    }

}
