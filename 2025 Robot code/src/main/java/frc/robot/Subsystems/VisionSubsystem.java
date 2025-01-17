package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    Vision vision;
    boolean targetVisible = false;
    Rotation2d targetYaw;
    double targetRange = 0.0;

    public VisionSubsystem(Vision vision) {
        this.vision = vision;
        // Initialize vision components here
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double getXSpeed(int side) {
        ChassisSpeeds speeds = processImage(side);
        if (speeds != null) {
            return speeds.vxMetersPerSecond;
        } else {
            return 0.0;
        }
    }

    public double getYSpeed(int side) {
        ChassisSpeeds speeds = processImage(side);
        if (speeds != null) {
            return speeds.vyMetersPerSecond;
        } else {
            return 0.0;
        }
    }

    public double getOmegaSpeed(int side) {
        ChassisSpeeds speeds = processImage(side);
        if (speeds != null) {
            return speeds.omegaRadiansPerSecond;
        } else {
            return 0.0;
        }
    }

    public int getFiducialId() {
        return vision.getCamera().getLatestResult().getBestTarget().fiducialId;
    }

    public ChassisSpeeds processImage(int Side) {
        var results = vision.getCamera().getAllUnreadResults();

        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == Side) {
                        Pose3d robotpose = vision.getEstimatedGlobalPose().get().estimatedPose;
                        Pose3d targetPose = Constants.Vision.kTagLayout.getTagPose(Side).get();
                        targetYaw = PhotonUtils.getYawToPose(robotpose.toPose2d(), targetPose.toPose2d());
                        targetRange = PhotonUtils.getDistanceToPose(robotpose.toPose2d(), targetPose.toPose2d());
                        SmartDashboard.putNumber("fowrad", (targetRange * Math.cos(targetYaw.getRadians())));
                        SmartDashboard.putNumber("Strafe", targetRange * Math.sin(targetYaw.getRadians()));
                        ChassisSpeeds speeds = new ChassisSpeeds(
                                targetRange * Math.cos(targetYaw.getRadians()),
                                targetRange * Math.sin(targetYaw.getRadians()),
                                0);
                        return speeds;
                    }
                }
            }
        }

        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
        return speeds;
    }
}