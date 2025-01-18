package frc.robot.commands.automation.reef;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.drive.DriveTrain;
import frc.robot.commands.reef.IntakeCoral;
import frc.robot.commands.reef.L1;
import frc.robot.Constants.*;
import frc.robot.commands.automation.AutoAlignment;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.Supplier;

public class PathFindToPoseAndPlaceSequence extends AutoAlignment {
    public PathFindToPoseAndPlaceSequence(
            DriveTrain driveSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            frc.robot.Subsystems.VisionSubsystem vision,
            Supplier<Translation2d> robotPrepareToPlacePositionSupplier,
            Supplier<Translation2d> robotMidPrepareToPlacePositionSupplier,
            Supplier<Translation2d> robotPlacingPositionSupplier,
            int level,
            int face
    ) {
        super(
            driveSubsystem, 
            () -> {
                final Translation2d displacementToTarget = FieldConstants.Reef.centerFaces[face].getTranslation().minus(robotPlacingPositionSupplier.get());
                return new Pose2d(
                        robotPrepareToPlacePositionSupplier.get(),
                        displacementToTarget.getAngle()
                );
            },
            () -> new Pose2d(
                    robotMidPrepareToPlacePositionSupplier.get(),
                    new Rotation2d(0)
            ),
            () -> new Pose2d(
                robotPlacingPositionSupplier.get(),
                frc.robot.Constants.FieldConstants.Reef.centerFaces[face].getRotation()
            ),
            new Pose2d(0.03, 0.03, Rotation2d.fromDegrees(2)), 
            0.75,
            new L1(elevatorSubsystem),
            new IntakeCoral(elevatorSubsystem, vision)
        );
                
        super.addRequirements(driveSubsystem, elevatorSubsystem);
    }

    private static boolean isChassisSlowEnough(DriveTrain driveSubsystem) {
        final ChassisSpeeds vel =  driveSubsystem.getSpeed();
        return Math.hypot(vel.vxMetersPerSecond, vel.vyMetersPerSecond) < 0.7
                && Math.abs(vel.omegaRadiansPerSecond) < Math.toRadians(50);
    }
}
