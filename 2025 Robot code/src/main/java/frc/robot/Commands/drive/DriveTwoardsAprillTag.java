package frc.robot.commands.drive;

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

public class DriveTwoardsAprillTag extends Command{
    private ProfiledPIDController drivePidController = 
    new ProfiledPIDController(
          0,
          0,
          0,
          new TrapezoidProfile.Constraints(
              4, 3.5));
    private ProfiledPIDController strafePidController = 
    new ProfiledPIDController(
        0,
        0, 
        0,
        new TrapezoidProfile.Constraints(
            4, 3.5));

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);
    Vision vision;
    DriveTrain driveTrain;

    public DriveTwoardsAprillTag(Vision vision, DriveTrain driveTrain) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        addRequirements(vision, driveTrain);
    }

    @Override
    public void execute() {
        double disitance = PhotonUtils.calculateDistanceToTargetMeters(Units.inchesToMeters(23.5), Constants.Vision.kTagLayout.getTagPose(vision.getTargets().fiducialId).get().getZ(), 0, 0);
        chassisSpeeds.vyMetersPerSecond = drivePidController.calculate(vision.getEstimatedGlobalPose().get().estimatedPose.getX(), disitance);
        chassisSpeeds.vxMetersPerSecond = strafePidController.calculate(vision.getTargets().getYaw(), 0);
        chassisSpeeds.omegaRadiansPerSecond = 0;
        driveTrain.driveRobotRelative(chassisSpeeds);
    }
}
