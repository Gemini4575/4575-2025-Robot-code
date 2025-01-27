package frc.robot.commands.automation.reef;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AprilTagToSide;
import frc.lib.util.SideOffSetCalculation;
import frc.lib.util.PoseTools;
import frc.robot.Constants.FieldConstants;
import frc.robot.Subsystems.NoraArmSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.drive.DriveTrain;
import frc.robot.utils.MapleShooterOptimization;

public class GoToReefSideAndPlace extends Command {

        private NoraArmSubsystem elevator;
        private DriveTrain driveTrain;
        private frc.robot.Subsystems.Vision vision;
        private VisionSubsystem visionSubsystem;
        private AprilTagToSide aprilTagToSide;
        private SideOffSetCalculation sideOffSetCalculation;
        private PoseTools translation3dToTranslation2d;
        private int level = 0;
        private int side = 0;

        private PathFindToPoseAndPlaceSequence pathFindToPoseAndPlaceSequence;

        public GoToReefSideAndPlace(NoraArmSubsystem elevator, DriveTrain driveTrain,
                        frc.robot.Subsystems.Vision vision,
                        int level, int side, VisionSubsystem visionSubsystem) {
                this.elevator = elevator;
                this.driveTrain = driveTrain;
                this.vision = vision;
                this.level = level;
                this.side = side;
                this.visionSubsystem = visionSubsystem;
                aprilTagToSide = new AprilTagToSide();
                sideOffSetCalculation = new SideOffSetCalculation();
                translation3dToTranslation2d = new PoseTools();

                addRequirements(elevator, driveTrain, vision, visionSubsystem);
        }

        @Override
        public void initialize() {
                pathFindToPoseAndPlaceSequence = new PathFindToPoseAndPlaceSequence(
                                driveTrain,
                                elevator,
                                visionSubsystem,
                                () -> sideOffSetCalculation.calculate(FieldConstants.Reef.centerFaces[side], side)
                                                .getTranslation(),
                                () -> {
                                        if (side == 2 || side == 3 || side == 4) {
                                                return new Translation2d(6.361, 4.092);
                                        } else if (side == 1) {
                                                return FieldConstants.Reef.RedBargeSidePassthough.getTranslation();
                                        } else {
                                                return FieldConstants.Reef.BlueBargeSidePassthough.getTranslation();
                                        }
                                },
                                () -> translation3dToTranslation2d.calculate(vision.getTagCamera().getLatestResult().getBestTarget().bestCameraToTarget.getTranslation()),
                                level,
                                aprilTagToSide.calculate(
                                                vision.getTagCamera().getLatestResult().getBestTarget().fiducialId));

                // Initialization code here
        }

        @Override
        public void execute() {
                pathFindToPoseAndPlaceSequence.schedule();
        }

        @Override
        public void end(boolean interrupted) {
                // Code to stop the elevator
        }
}
