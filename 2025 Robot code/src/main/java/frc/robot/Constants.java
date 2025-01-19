package frc.robot;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.RotationsToInch;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Subsystems.drive.DriveTrain;
import frc.robot.utils.MechanismControl.MaplePIDController;

public class Constants {
    public static RotationsToInch rotationsToInch = new RotationsToInch();

    public enum RobotMode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    public static final String ROBOT_NAME = "5516-2024-OffSeason";

    // avoid typo errors
    public static final class LogConfigs {
        public static final String
                SYSTEM_PERFORMANCE_PATH = "SystemPerformance/",
                PHYSICS_SIMULATION_PATH = "MaplePhysicsSimulation/",
                APRIL_TAGS_VISION_PATH = "Vision/AprilTags/",
                SHOOTER_PATH = "Shooter/";
    }

    public static final class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class Reef {
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d RedBargeSidePassthough = new Pose2d(
        4.496,
        1.264,
        Rotation2d.fromDegrees(0));
    public static final Pose2d BlueBargeSidePassthough = new Pose2d(
        4.475,
        6.796,
        Rotation2d.fromDegrees(0));
    

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefHeight, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

    static {
      // Initialize faces
    //   centerFaces[-1] =
    //       new Pose2d(
    //           Units.inchesToMeters(144.003),
    //           Units.inchesToMeters(158.500),
    //           Rotation2d.fromDegrees(180));
      centerFaces[0] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
      centerFaces[1] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
      centerFaces[2] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
      centerFaces[3] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
      centerFaces[4] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
      centerFaces[5] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
        Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
        for (var level : ReefHeight.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
        }
        // branchPositions.add((face * 2) + 1, fillRight);
        // branchPositions.add((face * 2) + 2, fillLeft);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public enum ReefHeight {
    L4(Units.inchesToMeters(72), -90),
    L3(Units.inchesToMeters(47.625), -35),
    L2(Units.inchesToMeters(31.875), -35),
    L1(Units.inchesToMeters(18), 0);

    ReefHeight(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // in degrees
    }

    public final double height;
    public final double pitch;
  }
}

    public static final class DriveConfigs {
        public static final double DRIVE_TRANSLATIONAL_SENSITIVITY = 1;
        public static final double DRIVE_ROTATIONAL_SENSITIVITY = 1;

        public static final double nonUsageTimeResetWheels = 5;

        public static final double deadBandWhenOtherAxisEmpty = 0.02;
        public static final double deadBandWhenOtherAxisFull = 0.1;
        public static final double linearSpeedInputExponent = 1.6;
        public static final double rotationSpeedInputExponent = 2;

        /** the amount of time that the chassis needs to accelerate to the maximum linear velocity */
        public static final double linearAccelerationSmoothOutSeconds = 0.1;
        /** the amount of time that the chassis needs to accelerate to the maximum angular velocity */
        public static final double angularAccelerationSmoothOutSeconds = 0.1;

        public static final double timeActivateRotationMaintenanceAfterNoRotationalInputSeconds = 0.3;
    }

    public static final class SwerveDriveChassisConfigs {
        public enum SwerveDriveType {
            REV,
            CTRE_ON_RIO,
            CTRE_ON_CANIVORE
        }
        public static final SwerveDriveType SWERVE_DRIVE_TYPE = SwerveDriveType.CTRE_ON_CANIVORE;

        public static final String CHASSIS_CANBUS = "ChassisCanivore";

        public static final int ODOMETRY_CACHE_CAPACITY = 10;
        public static final double ODOMETRY_FREQUENCY = 250;
        public static final double ODOMETRY_WAIT_TIMEOUT_SECONDS = 0.05;

        public static final MaplePIDController.MaplePIDConfig chassisRotationalPIDConfig = new MaplePIDController.MaplePIDConfig(
                Math.toRadians(360),
                Math.toRadians(65),
                0.02,
                Math.toRadians(3),
                0,
                true,
                0
        );
        public static final TrapezoidProfile.Constraints chassisRotationalConstraints = new TrapezoidProfile.Constraints(
                Math.toRadians(360),
                Math.toRadians(ChassisDefaultConfigs.DEFAULT_MAX_ANGULAR_ACCELERATION_DEGREES_PER_SECOND_SQUARE)
        );

        public static final MaplePIDController.MaplePIDConfig chassisTranslationPIDConfig = new MaplePIDController.MaplePIDConfig(
                3,
                1,
                0.05,
                0.05,
                0.05,
                false,
                0
        );

        public static final MaplePIDController.MaplePIDConfig chassisTranslationPIDConfigPathFollowing = new MaplePIDController.MaplePIDConfig(
                2,
                1.2,
                0,
                0.03,
                0,
                false,
                0
        );
    }

    public static final class ChassisDefaultConfigs {
        public static final int DEFAULT_GYRO_PORT = 0;
        public static final double DEFAULT_GEAR_RATIO = 6.12;
        public static final double DEFAULT_WHEEL_RADIUS_METERS = 0.051; // 2 inch
        public static final double DEFAULT_HORIZONTAL_WHEELS_MARGIN_METERS = 0.53;
        public static final double DEFAULT_VERTICAL_WHEELS_MARGIN_METERS = 0.53;
        public static final double DEFAULT_MAX_VELOCITY_METERS_PER_SECOND = 4.172; // calculated from Choreo (Kraken x60 motor, 6.12 gear ratio, 55kg robot mass)
        public static final double DEFAULT_MAX_ACCELERATION_METERS_PER_SQUARED_SECOND = 10.184; // calculated from Choreo (Kraken x60 motor, 6.12 gear ratio, 55kg robot mass)
        public static final double DEFAULT_MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND = 540;
        public static final double DEFAULT_MAX_ANGULAR_ACCELERATION_DEGREES_PER_SECOND_SQUARE = 540;
    }

    public static final class SwerveModuleConfigs {
        public static final MaplePIDController.MaplePIDConfig steerHeadingCloseLoopConfig = new MaplePIDController.MaplePIDConfig(
                0.8,
                Math.toRadians(60),
                0.02,
                Math.toRadians(1.5),
                0,
                true,
                0
        );
        public static final double STEERING_CURRENT_LIMIT = 20;
        public static final double DRIVING_CURRENT_LIMIT = 40;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

        public static final SimpleMotorFeedforward DRIVE_OPEN_LOOP = new SimpleMotorFeedforward(0.05, 2.5);
    }

    public static final class RobotPhysicsSimulationConfigs {
        public static final int SIM_ITERATIONS_PER_ROBOT_PERIOD = 5;

        /* Swerve Module Simulation */
        public static final double DRIVE_MOTOR_FREE_FINAL_SPEED_RPM = 4800;
        public static final DCMotor
                DRIVE_MOTOR = DCMotor.getKrakenX60(1),
                STEER_MOTOR = DCMotor.getFalcon500(1);
        public static final double DRIVE_WHEEL_ROTTER_INERTIA = 0.012;
        public static final double STEER_INERTIA = 0.015;
        public static final double STEER_GEAR_RATIO = 150.0 / 7.0;

        public static final double FLOOR_FRICTION_ACCELERATION_METERS_PER_SEC_SQ = 15;
        public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = Math.toRadians(1200);
        public static final double TIME_CHASSIS_STOPS_ROTATING_NO_POWER_SEC = 0.3;
        public static final double DEFAULT_ROBOT_MASS = 40;
        public static final double DEFAULT_BUMPER_WIDTH_METERS = Units.inchesToMeters(34.5);
        public static final double DEFAULT_BUMPER_LENGTH_METERS = Units.inchesToMeters(36);

        /* https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction */
        public static final double ROBOT_BUMPER_COEFFICIENT_OF_FRICTION = 0.85;
        /* https://en.wikipedia.org/wiki/Coefficient_of_restitution */
        public static final double ROBOT_BUMPER_COEFFICIENT_OF_RESTITUTION = 0.05;

        /* Gyro Sim */
        public static final double GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ = 100;
        public static final double SKIDDING_AMOUNT_AT_THRESHOLD_RAD = Math.toRadians(1.2);
        /*
        * https://store.ctr-electronics.com/pigeon-2/
        * for a well-installed one with vibration reduction, only 0.4 degree
        * but most teams just install it directly on the rigid chassis frame (including my team :D)
        * so at least 1.2 degrees of drifting in 1 minutes for an average angular velocity of 60 degrees/second
        * which is the average velocity during normal swerve-circular-offense
        * */
        public static final double NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD = Math.toRadians(1.2);
        public static final double AVERAGE_VELOCITY_RAD_PER_SEC_DURING_TEST = Math.toRadians(60);
    }

    public static final class VisionConfigs {
        public static final AprilTagFieldLayout fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        /* default standard error for vision observation, if only one apriltag observed */
        public static final double
                TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION = 0.6,
                ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION = 0.5,

                // only do odometry calibration if translational standard error if it is not greater than
                TRANSLATIONAL_STANDARD_ERROR_THRESHOLD = 0.5,
                // only do gyro calibration if rotational standard error is very, very small
                ROTATIONAL_STANDARD_ERROR_THRESHOLD = Math.toRadians(5),

                ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS = 0.02,
                // we trust the IMU very much (recommend 0.1 for Pigeon2, 0.5 for NavX)
                GYRO_ROTATIONAL_STANDARD_ERROR_RADIANS = Math.toRadians(0.1);
    }

    public static final class PitchConfigs {
        public static final double GEAR_RATIO = 166.66;
        public static final double PITCH_LOWEST_ROTATION_RAD = Math.toRadians(24);
        public static final double PITCH_HIGHER_LIMIT_RAD = Math.toRadians(100);

        public static final double PITCH_KS = 0.03;
        public static final double PITCH_KG = 0.1;
        public static final double PITCH_KV = 3.2;
        public static final double PITCH_KA = 0.01;

        public static final MaplePIDController.MaplePIDConfig PITCH_PID = new MaplePIDController.MaplePIDConfig(
                7.5,
                Math.toRadians(26),
                0,
                Math.toRadians(2),
                0.05,
                false,
                0
        );

        public static final TrapezoidProfile.Constraints PITCH_PROFILE_CONSTRAIN = new TrapezoidProfile.Constraints(
                Math.toRadians(360), Math.toRadians(720)
        );
    }

    public static final class ShooterConfigs {
        public static final double[] ks = new double[] {0.05, 0.05};
        public static final double[] kv = new double[] {0.11613, 0.1145};
        public static final double kv_sim = 0.17;
        public static final double[] ka = new double[] {0.026625, 0.028551};

        public static final TrapezoidProfile.Constraints SPEED_RPM_CONSTRAINS = new TrapezoidProfile.Constraints(
                6000/0.5, 6000/0.3
        );

        public static final double TOLERANCE_RPM = 65;
        public static final MaplePIDController.MaplePIDConfig FLYWHEEL_PID_CONFIG_REV_PER_SEC = new MaplePIDController.MaplePIDConfig(
                7,
                30,
                0,
                2,
                0, false, 0
        );
    }

    public static Rotation2d toCurrentAllianceRotation(Rotation2d rotationAtBlueSide) {
        final Rotation2d
                yAxis = Rotation2d.fromDegrees(90),
                differenceFromYAxisAtBlueSide = rotationAtBlueSide.minus(yAxis),
                differenceFromYAxisNew = differenceFromYAxisAtBlueSide.times(isSidePresentedAsRed() ? -1:1);
        return yAxis.rotateBy(differenceFromYAxisNew);
    }

    public static Translation2d toCurrentAllianceTranslation(Translation2d translationAtBlueSide) {
        if (isSidePresentedAsRed())
            return new Translation2d(
                    FieldConstants.fieldWidth - translationAtBlueSide.getX(),
                    translationAtBlueSide.getY()
            );
        return translationAtBlueSide;
    }

    public static Translation3d toCurrentAllianceTranslation(Translation3d translation3dAtBlueSide) {
        final Translation2d translation3dAtCurrentAlliance = toCurrentAllianceTranslation(translation3dAtBlueSide.toTranslation2d());
        if (isSidePresentedAsRed())
            return new Translation3d(
                    translation3dAtCurrentAlliance.getX(),
                    translation3dAtCurrentAlliance.getY(),
                    translation3dAtBlueSide.getZ()
            );
        return translation3dAtBlueSide;
    }

    public static Pose2d toCurrentAlliancePose(Pose2d poseAtBlueSide) {
        return new Pose2d(
                toCurrentAllianceTranslation(poseAtBlueSide.getTranslation()),
                toCurrentAllianceRotation(poseAtBlueSide.getRotation())
        );
    }

    public static PathPlannerPath toCurrentAlliancePath(PathPlannerPath pathAtBlueAlliance) {
        return isSidePresentedAsRed() ? pathAtBlueAlliance.flipPath() : pathAtBlueAlliance;
    }

    public static boolean isSidePresentedAsRed() {
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get().equals(DriverStation.Alliance.Red);
    }

    public static Rotation2d getDriverStationFacing() {
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
            case Red -> new Rotation2d(Math.PI);
            case Blue -> new Rotation2d(0);
        };
    }
    public static final double stickDeadband = 0.3;
    
    public static final class Vision {
        public static final String kTagCameraName = "Arducam1";
        public static final String kAlgaeCameraName = "ArducamColor";
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
        //TODO update with real value
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
                // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        private static final int ALGAE_CAM_WIDTH = 1200;
        private static final int ALGAE_CAM_HEIGHT = 800;
        public static final int ALGAE_CAM_AREA = ALGAE_CAM_HEIGHT * ALGAE_CAM_WIDTH;
    
        public static final double ALGAE_CAM_FOCAL_LENGTH = 70;
    
        public static final double ALGAE_REAL_DIAMETER = Units.inchesToMeters(16.25); //meters
    }

    public static final class SwerveConstants {
         // these are distance from the center to the wheel in meters. .381 is 1.25 feet or 16 inches
        // swerve drive has 35.5 inch diagonals
/*
        private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
*/
/*  These numbers are for 28.5 swerve 
public static final Translation2d m_frontLeftLocation = new Translation2d(0.4445, 0.4445);
public static final Translation2d m_frontRightLocation = new Translation2d(0.4445, -0.4445);
public static final Translation2d m_backLeftLocation = new Translation2d(-0.4445, 0.4445);
public static final Translation2d m_backRightLocation = new Translation2d(-0.4445, -0.4445);

/*  These numbers are for 29.5 swerve
0.45085
private final Translation2d m_frontLeftLocation = new Translation2d(0.45085, 0.45085);
private final Translation2d m_frontRightLocation = new Translation2d(0.45085, -0.45085);
private final Translation2d m_backLeftLocation = new Translation2d(-0.45085, 0.45085);
private final Translation2d m_backRightLocation = new Translation2d(-0.45085, -0.45085);

//These numbers are for the weird rectangle swerve
//0.2032 X
//0.2794 Y
        public static final Translation2d m_frontLeftLocation = new Translation2d(0.2032, 0.2794);
        public static final Translation2d m_frontRightLocation = new Translation2d(0.2032, -0.2794);
        public static final Translation2d m_backLeftLocation = new Translation2d(-0.2032, 0.2794);
        public static final Translation2d m_backRightLocation = new Translation2d(-0.2032, -0.2794);
        */
        // These number are for a 25 by 25 swerve
        // 0.33333
        public static final Translation2d m_frontLeftLocation = new Translation2d(0.3333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333, 0.33333);
        public static final Translation2d m_frontRightLocation = new Translation2d(0.33333, -0.33333);
        public static final Translation2d m_backLeftLocation = new Translation2d(-0.33333, 0.33333);
        public static final Translation2d m_backRightLocation = new Translation2d(-0.33333, -0.33333);

        /* Ints */
            public static final int kEncoderResolution = 4096;
        /* Doubles */
            public static final double kWheelRadius = 0.0508;
            public static final double kModuleMaxAngularVelocity = DriveTrain.kMaxAngularSpeed;
            public static final double kModuleMaxAngularAcceleration = 18.85;//4 * Math.PI; // radians per second squared
            // FWF - stole this from 6328's code, each gear reduction written out. Final is 6.75. 39.37 converts inches to meters so we can be european fancy
            //private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
            //public static final double driveAfterEncoderReduction = (4.0 / 39.37) * Math.PI * (1/6.75);
//            public static final double driveAfterEncoderReduction = 0.0788114854;   // FWF this is the above calc * 1.6667 to see if the auto distance changes

            public static final double turnAfterEncoderReduction = -1 * (7/150);
        
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 0;
            public static final double angleOffset = 3.201315307;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 1;
            public static final double angleOffset = 5.333449307;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 2;
            public static final double angleOffset = 0.2082833072;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 3;
            public static final double angleOffset = 3.769512307;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID);
        }
    }

    public final static class JoystickConstants{
        public final static int DRIVER_USB = 0;
        public final static int OPERATOR_USB = 1;
        public final static int TEST_USB = 2;
        
        public final static int LEFT_Y_AXIS = 1;
        public final static int LEFT_X_AXIS = 0;
        public final static int RIGHT_X_AXIS = 4;
        public final static int RIGHT_Y_AXIS = 5;
    
        public final static int GREEN_BUTTON = 1;
        public final static int RED_BUTTON = 2;
        public final static int YELLOW_BUTTON = 4;
        public final static int BLUE_BUTTON = 3;
    
        public final static int LEFT_TRIGGER = 2;
        public final static int RIGHT_TRIGGER = 3;
        public final static int LEFT_BUMPER = 5;
        public final static int RIGHT_BUMPER = 6;
    
        public final static int BACK_BUTTON = 7;
        public final static int START_BUTTON = 8;
    
        public final static int POV_UP = 0;
        public final static int POV_RIGHT = 90;
        public final static int POV_DOWN = 180;
        public final static int POV_LEFT = 270;
    }

    public final static class ArmConstants {
        /* Ints */
            public final static int ARM_MOTOR = 13;
            public final static int TOP_Wheel = 0;
            public final static int BOTTOM_Wheel = 1;
            public final static int CoralSensor = 0;
        /* Doubles */
            public final static double P = 0.0;
            public final static double I = 0.0;
            public final static double D = 0.0;
            public final static double L1Position = rotationsToInch.calculateTicks(I, I);
            public final static double L2Position = 0.0;
            public final static double L3Position = 0.0;
            public final static double L4Position = 0.0;
            public final static double CoralStationPosition = 0.0;
            public final static double maxSpeed = 0.0;
            public final static double maxAcceleration = 0.0;
            public final static double IntakeSpeed = 0.5;
            
    }

    public final static class ClimbingConstants {
        /* Ints */
            public final static int ClimbingMotor1 = 14;
            public final static int ClimbingMotor2 = 15;
            //TODO get the real values
            public final static int ClimbingMotorPoseition = 0;
        /* Doubles */
            public final static double ClimbingSpeed = 0.5;
    }
}