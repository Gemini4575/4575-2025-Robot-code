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
    private boolean firstTime = false;
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
        if(vision.getTargets().area > 10.64) {
            chassisSpeeds.vyMetersPerSecond = 0; 
            this.end(false); 
        }else {
            chassisSpeeds.vyMetersPerSecond = 0.3;
        }
        if(Math.abs(vision.getTargets().getYaw()) > 10) {
            chassisSpeeds.vxMetersPerSecond = -((vision.getTargets().getYaw() - 1) / 100);
        } else {
            chassisSpeeds.vxMetersPerSecond = 0;
        }
        chassisSpeeds.omegaRadiansPerSecond = 0;
        driveTrain.driveRobotRelative(chassisSpeeds);
    }
}
