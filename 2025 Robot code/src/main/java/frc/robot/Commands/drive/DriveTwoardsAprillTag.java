package frc.robot.commands.drive;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.DriveTrain;

public class DriveTwoardsAprillTag extends Command {
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    Vision vision;
    DriveTrain driveTrain;

    public DriveTwoardsAprillTag(Vision vision, DriveTrain driveTrain) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        addRequirements(vision, driveTrain);
    }
    boolean isFinished;
    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void execute() {
        if (vision.getTargets().area > 5 || vision.getTargets() == null) {
            chassisSpeeds.vxMetersPerSecond = 0;
            chassisSpeeds.vyMetersPerSecond = 0;
            isFinished = true;
        } else {
            chassisSpeeds.vxMetersPerSecond = -0.05;
        }
        if (Math.abs(vision.getTargets().getYaw()) > 10) {
            chassisSpeeds.vyMetersPerSecond = ((vision.getTargets().getYaw() - 1) / 1000);
        } else {
            chassisSpeeds.vyMetersPerSecond = 0;
        }
        chassisSpeeds.omegaRadiansPerSecond = 0;
        driveTrain.driveRobotRelative(chassisSpeeds);
    }
}
