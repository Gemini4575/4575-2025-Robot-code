package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.drive.DriveTrain;
import frc.robot.utils.MechanismControl.MaplePIDController;
import frc.robot.utils.MechanismControl.MapleProfiledPIDController;

import java.util.function.Supplier;

public class DriveToPose extends Command {
    private final Supplier<Pose2d> desiredPose1;
    private final Supplier<Pose2d> desiredPose2;
    private final DriveTrain driveSubsystem;
    private final HolonomicDriveController positionController;

    private final double speedConstrainMPS;
    private final Pose2d tolerance;
    private boolean firstPoseReached;

    public DriveToPose(DriveTrain driveSubsystem, Supplier<Pose2d> desiredPose1, Supplier<Pose2d> desiredPose2) {
        this(driveSubsystem, desiredPose1, desiredPose2, new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(10)), 3);
    }

    public DriveToPose(DriveTrain driveSubsystem, Supplier<Pose2d> desiredPose1, Supplier<Pose2d> desiredPose2, Pose2d tolerance, double speedConstrainMPS) {
        this.desiredPose1 = desiredPose1;
        this.desiredPose2 = desiredPose2;
        this.driveSubsystem = driveSubsystem;
        this.positionController = createPositionController();
        this.tolerance = tolerance;
        this.speedConstrainMPS = speedConstrainMPS;
        this.firstPoseReached = false;

        super.addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        getFeedBackSpeeds();
    }

    @Override
    public void execute() {
        ChassisSpeeds feedBackSpeeds = getFeedBackSpeeds();
        final double feedBackSpeedMagnitude = Math.hypot(feedBackSpeeds.vxMetersPerSecond, feedBackSpeeds.vyMetersPerSecond);
        if (feedBackSpeedMagnitude < speedConstrainMPS)
            feedBackSpeeds = feedBackSpeeds.times(speedConstrainMPS / feedBackSpeedMagnitude);
        driveSubsystem.driveRobotRelative(feedBackSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    private ChassisSpeeds getFeedBackSpeeds() {
        if (!firstPoseReached) {
            return positionController.calculate(driveSubsystem.getPose(), desiredPose1.get(), 0, desiredPose1.get().getRotation());
        } else {
            return positionController.calculate(driveSubsystem.getPose(), desiredPose2.get(), 0, desiredPose2.get().getRotation());
        }
    }

    @Override
    public boolean isFinished() {
        final Pose2d desiredPose = firstPoseReached ? desiredPose2.get() : desiredPose1.get();
        final Pose2d currentPose = driveSubsystem.getPose();
        final ChassisSpeeds speeds = driveSubsystem.getSpeed();
        boolean isCurrentPoseFinished = Math.abs(desiredPose.getX() - currentPose.getX()) < tolerance.getX()
                && Math.abs(desiredPose.getY() - currentPose.getY()) < tolerance.getY()
                && Math.abs(desiredPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees()) < tolerance.getRotation().getDegrees()
                && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.8
                && Math.abs(speeds.omegaRadiansPerSecond) < Math.toRadians(30);

        if (isCurrentPoseFinished) {
            if (!firstPoseReached) {
                firstPoseReached = true;
            } else {
                return true;
            }
        }
        return false;
    }

    public static HolonomicDriveController createPositionController() {
        return null;// return new HolonomicDriveController(
        //         new MaplePIDController(Constants.SwerveDriveChassisConfigs.chassisTranslationPIDConfig),
        //         new MaplePIDController(Constants.SwerveDriveChassisConfigs.chassisTranslationPIDConfig),
        //         new MapleProfiledPIDController(Constants.SwerveDriveChassisConfigs.chassisRotationalPIDConfig, Constants.SwerveDriveChassisConfigs.chassisRotationalConstraints)
        // );
    }
}
