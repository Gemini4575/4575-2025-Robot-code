package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import com.google.flatbuffers.Table;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.Config.MapleInterpolationTable;
import frc.robot.utils.Config.MapleInterpolationTable.Variable;

public class MapleShooterOptimization {
    public static final class ShooterState {
        public final double shooterAngleDegrees;
        public final double shooterAngleChangeRateDegreesPerSecond;
        public final double shooterRPM;
        public final double shooterRPMChangeRateRPMPerSeconds;

        private ShooterState(double shooterAngleDegrees, double shooterAngleChangeRateDegreesPerSecond, double shooterRPM, double shooterRPMChangeRateRPMPerSeconds) {
            this.shooterAngleDegrees = shooterAngleDegrees;
            this.shooterAngleChangeRateDegreesPerSecond = shooterAngleChangeRateDegreesPerSecond;
            this.shooterRPM = shooterRPM;
            this.shooterRPMChangeRateRPMPerSeconds = shooterRPMChangeRateRPMPerSeconds;
        }

        @Override
        public String toString() {
            return "ShootingState{" +
                    "shooterAngleDegrees=" + shooterAngleDegrees +
                    ", shooterAngleChangeRateDegreesPerSecond=" + shooterAngleChangeRateDegreesPerSecond +
                    ", shooterRPM=" + shooterRPM +
                    ", shooterRPMChangeRateRPMPerSeconds=" + shooterRPMChangeRateRPMPerSeconds +
                    '}';
        }

        public void log(String logPath) {
            Logger.recordOutput(logPath + "ShooterAngleDegrees", shooterAngleDegrees);
            Logger.recordOutput(logPath + "ShooterAngleChangeRateDegreesPerSecond", shooterAngleChangeRateDegreesPerSecond);
            Logger.recordOutput(logPath + "ShooterRPM", shooterRPM);
            Logger.recordOutput(logPath + "ShooterRPMChangeRateRPMPerSeconds", shooterRPMChangeRateRPMPerSeconds);
        }
    }

    private final String name;
    private final MapleInterpolationTable table;
    private final double minShootingDistance, maxShootingDistance;
    public MapleShooterOptimization(String name, double[] distancesToTargetsMeters, double[] shooterAngleDegrees, double[] shooterRPM, double[] projectileFlightTimeSeconds) {
        this(name, new MapleInterpolationTable(
                name,
                new Variable("Distance-To-Target", distancesToTargetsMeters),
                new Variable("Shooter-Angle-Degrees", shooterAngleDegrees),
                new Variable("Shooter-RPM", shooterRPM),
                new Variable("Flight-Time", projectileFlightTimeSeconds)
        ));
    }

    private MapleShooterOptimization(String name, MapleInterpolationTable table) {
        this.name = name;
        this.table = table;

        this.minShootingDistance = table.minX;
        this.maxShootingDistance = table.maxX;
    }
    


    public double getFlightTimeSeconds(Translation2d targetPosition, Translation2d robotPosition) {
        final double distanceToTargetMeters = targetPosition.getDistance(robotPosition);
        return table.interpolateVariable("Flight-Time", distanceToTargetMeters);
    }

    public Rotation2d getShooterFacing(Translation2d targetPosition, Translation2d robotPosition, ChassisSpeeds robotVelocityFieldRelative) {
        final double flightTime = getFlightTimeSeconds(targetPosition, robotPosition);
        final Translation2d robotPositionAfterFlightTime = robotPosition.plus(new Translation2d(
                robotVelocityFieldRelative.vxMetersPerSecond * flightTime,
                robotVelocityFieldRelative.vyMetersPerSecond * flightTime
        ));

        return targetPosition.minus(robotPositionAfterFlightTime).getAngle();
    }
    
}
