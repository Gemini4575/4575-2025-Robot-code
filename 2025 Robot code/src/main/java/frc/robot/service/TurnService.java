package frc.robot.service;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveTrain;

public class TurnService {

    private static final double DRIFT_TOLERANCE = 10.0;

    private final DriveTrain driveTrain;
    private double degreesToTurn;
    private Pose2d startingPose;
    private double startingDegrees;
    private boolean completed = false;

    public TurnService(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    public void startTurning(double degreesToTurn, double radiansPerSec) {
        this.degreesToTurn = degreesToTurn;
        startingPose = driveTrain.getPose();
        startingDegrees = driveTrain.getYaw();
        completed = false;
        driveTrain.drive(0, 0, radiansPerSec, false);
    }

    public boolean keepTurning() {
        if (!completed) {
            var degreesTurnedSoFar = calculateTurn();
            SmartDashboard.putNumber("Distance driven", degreesTurnedSoFar);
            if (excessAngle(degreesTurnedSoFar) >= 0) {
                System.out.println("INFO: turn for " + degreesToTurn + " degrees is completed at " + degreesTurnedSoFar
                        + " degrees");
                driveTrain.stop();
                completed = true;
            }
        }
        return !completed;
    }

    private double calculateTurn() {
        var currentPose = driveTrain.getPose();
        var poseAngleDiff = currentPose.getRotation().minus(startingPose.getRotation()).getDegrees();
        var gyroDiff = driveTrain.getYaw() - startingDegrees;
        if (Math.abs(poseAngleDiff - gyroDiff) > DRIFT_TOLERANCE) {
            System.out.println("WARN: pose estimate and gyro diff drifted by " + Math.abs(poseAngleDiff - gyroDiff));
        }
        var excessPose = excessAngle(poseAngleDiff);
        if (poseAngleDiff > 0) {
            System.out.println("WARN: pose estimate turned past goal of " + degreesToTurn + " by " + excessPose);
        }
        return gyroDiff;
    }

    private double excessAngle(double angle) {
        return degreesToTurn > 0 ? angle - degreesToTurn : degreesToTurn - angle;
    }

}
