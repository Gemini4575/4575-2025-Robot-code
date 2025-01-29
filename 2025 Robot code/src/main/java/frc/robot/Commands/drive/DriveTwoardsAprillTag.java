package frc.robot.commands.drive;

import org.opencv.core.Mat.Tuple2;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.drive.DriveTrain;

public class DriveTwoardsAprillTag extends Command {
    private boolean firstTime = false;
    private ProfiledPIDController drivePidController = new ProfiledPIDController(
            0,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    4, 3.5));
    private ProfiledPIDController strafePidController = new ProfiledPIDController(
            0,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    4, 3.5));

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
