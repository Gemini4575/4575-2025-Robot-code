package frc.robot.commands.automation.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AprilTagToSide;
import frc.lib.util.SideOffSetCalculation;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Vision;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.drive.DriveTrain;
import frc.robot.Subsystems.vision.apriltags.AprilTagVision;
import frc.robot.utils.MapleShooterOptimization;

public class GoToReefSideAndPlace extends Command {

    private ElevatorSubsystem elevator;
    private DriveTrain driveTrain;
    private MapleShooterOptimization mapleShooterOptimization;
    private frc.robot.Subsystems.Vision vision;
    private AprilTagToSide aprilTagToSide;
    private SideOffSetCalculation sideOffSetCalculation;
    private int level = 0;

    private PathFindToPoseAndPlaceSequence pathFindToPoseAndPlaceSequence;

    public GoToReefSideAndPlace(ElevatorSubsystem elevator, DriveTrain driveTrain,
            MapleShooterOptimization mapleShooterOptimization, frc.robot.Subsystems.Vision vision,
            int level) {
        this.elevator = elevator;
        this.driveTrain = driveTrain;
        this.mapleShooterOptimization = mapleShooterOptimization;
        this.vision = vision;
        this.level = level;

        addRequirements(elevator, driveTrain, vision);
    }

    @Override
    public void initialize() {
        pathFindToPoseAndPlaceSequence = new PathFindToPoseAndPlaceSequence(
                mapleShooterOptimization,
                driveTrain,
                elevator,
                () -> {
                    if (aprilTagToSide.calculate(vision.getCamera().getLatestResult().getBestTarget().fiducialId) == 0
                            || aprilTagToSide
                                    .calculate(vision.getCamera().getLatestResult().getBestTarget().fiducialId) == 1
                            || aprilTagToSide
                                    .calculate(vision.getCamera().getLatestResult().getBestTarget().fiducialId) == 2) {
                        return sideOffSetCalculation
                                .calculate(
                                        FieldConstants.Reef.centerFaces[aprilTagToSide.calculate(
                                                vision.getCamera().getLatestResult().getBestTarget().fiducialId)],
                                        aprilTagToSide.calculate(
                                                vision.getCamera().getLatestResult().getBestTarget().fiducialId))
                                .getTranslation();
                    } else {
                        return sideOffSetCalculation
                                .calculate(
                                        FieldConstants.Reef.centerFaces[aprilTagToSide.calculate(
                                                vision.getCamera().getLatestResult().getBestTarget().fiducialId)],
                                        aprilTagToSide.calculate(
                                                vision.getCamera().getLatestResult().getBestTarget().fiducialId))
                                .getTranslation();
                    }
                },
                () -> {
                    if (aprilTagToSide.calculate(vision.getCamera().getLatestResult().getBestTarget().fiducialId) == 2
                            || aprilTagToSide
                                    .calculate(vision.getCamera().getLatestResult().getBestTarget().fiducialId) == 3
                            || aprilTagToSide
                                    .calculate(vision.getCamera().getLatestResult().getBestTarget().fiducialId) == 4) {
                        return new Translation2d();

                    } else if (aprilTagToSide
                            .calculate(vision.getCamera().getLatestResult().getBestTarget().fiducialId) == 1) {
                        return FieldConstants.Reef.RedBargeSidePassthough.getTranslation();
                    } else {
                        return FieldConstants.Reef.BlueBargeSidePassthough.getTranslation();
                    }
                },
                () -> vision.getEstimatedGlobalPose().get().estimatedPose.toPose2d().getTranslation(),
                level,
                aprilTagToSide.calculate(vision.getCamera().getLatestResult().getBestTarget().fiducialId));

        // Initialization code here
    }

    @Override
    public void execute() {
        pathFindToPoseAndPlaceSequence.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // Code to stop the elevator
    }
}
