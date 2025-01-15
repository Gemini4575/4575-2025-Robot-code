package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.commands.CommandOnFly;

import java.util.function.Supplier;

public class PathFindToPose extends Command{
    public PathFindToPose(DriveTrain driveSubsystem, Supplier<Pose2d> targetPose, double speedMultiplier) {
        AutoBuilder.pathfindToPose(
                targetPose.get(),
                driveSubsystem.getChassisConstrains()
        );
    }
}
