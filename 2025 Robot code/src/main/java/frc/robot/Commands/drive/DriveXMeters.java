package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.service.DriveService;
import frc.robot.subsystems.drive.DriveTrain;

public class DriveXMeters extends Command {
    
    double meters = 0.0;

    private final DriveService driveService;

    public DriveXMeters(DriveTrain driveTrain, double meters) {
        driveService = new DriveService(driveTrain);
        this.meters = meters;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveService.startDriving(1, meters, 0.005);
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
