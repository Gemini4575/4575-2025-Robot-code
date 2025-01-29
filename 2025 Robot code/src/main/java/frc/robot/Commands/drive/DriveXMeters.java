package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.DriveTrain;

public class DriveXMeters extends Command {
    DriveTrain s;
    double meters;

    public DriveXMeters(DriveTrain suDriveTrain, double meters) {
        this.s = suDriveTrain;
        this.meters = meters;
        if (meters == 0) {
            throw new RuntimeException("Meters must be over 0");
        }
        addRequirements(s);
    }

    @Override
    public void initialize() {
        s.first = true;
        isFinished = false;
    }
    boolean isFinished;
    
    @Override
    public boolean isFinished() {
        return isFinished;
    }


    @Override
    public void execute() {
        if (s.DriveMeters(meters)) {
           isFinished = true;
        }
    }
}
