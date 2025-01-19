// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import java.io.IOException;

// import org.springframework.beans.factory.annotation.Autowired;
// import org.springframework.stereotype.Component;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



/** Represents a swerve drive style drivetrain. */
// @Component
public class DriveTrain extends SubsystemBase {
  Field2d field = new Field2d();
  int ii = 0;
  public static final double kMaxSpeed = 12; // was 4.47 meters per second
  public static final double kMaxAngularSpeed = 4.41 * 2 * Math.PI; // was Math.PI for 1/2 rotation per second
  
  double Ptranslate = 10.0;
  double Itranslate = 0.0;
  double Dtranslate = 0.0;
  double Protate = 15.0;
  double Irotate = 2.0;
  double Drotate = 0.0;
  
  SendableChooser<Pose2d> ahhhhhh;

private final Translation2d m_frontLeftLocation = Constants.SwerveConstants.m_frontLeftLocation;
private final Translation2d m_frontRightLocation = Constants.SwerveConstants.m_frontRightLocation;
private final Translation2d m_backLeftLocation = Constants.SwerveConstants.m_backLeftLocation;
private final Translation2d m_backRightLocation = Constants.SwerveConstants.m_backRightLocation;


private final SwerveModule m_backLeft = new SwerveModule(Constants.SwerveConstants.Mod3.constants);
private final SwerveModule m_backRight = new SwerveModule(Constants.SwerveConstants.Mod2.constants);
private final SwerveModule m_frontLeft = new SwerveModule(Constants.SwerveConstants.Mod0.constants);
private final SwerveModule m_frontRight = new SwerveModule(Constants.SwerveConstants.Mod1.constants);

//  private final Gyro_EPRA m_gyro = new Gyro_EPRA();
private final AHRS m_gyro = new AHRS(NavXComType.kI2C);

private double xSpeed_cur;
private double ySpeed_cur;
private double rot_cur;
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  // private final SwerveDriveOdometry m_odometry =
  //     new SwerveDriveOdometry(
  //         m_kinematics,
  //         m_gyro.getRotation2d(),
  //         new SwerveModulePosition[] {
  //           m_frontLeft.getPosition(),
  //           m_frontRight.getPosition(),
  //           m_backLeft.getPosition(),
  //           m_backRight.getPosition()
  //         },
  //         new Pose2d(new Translation2d(),new Rotation2d(Units.degreesToRadians(180)))
  //         );

  private final SwerveDrivePoseEstimator poseEstimator;

  // private final DifferentialDrivetrainSim simDrive;
  // @Autowired
  // private MetricsProvider metricsProvider;

  public DriveTrain() {
    // SmartDashboard.putNumber("P rotate", Protate);
    // SmartDashboard.putNumber("D rotate", Drotate);
    // SmartDashboard.putNumber("I rotate", Irotate);

    // SmartDashboard.putNumber("P translate", Ptranslate);
    // SmartDashboard.putNumber("D translate", Dtranslate);
    // SmartDashboard.putNumber("I translate", Itranslate);
/*
    simDrive = new DifferentialDrivetrainSim(
                DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
                7.29, // 7.29:1 gearing reduction.
                7.5, // MOI of 7.5 kg m^2 (from CAD model).
                60.0, // The mass of the robot is 60 kg.
                Units.inchesToMeters(3), // The robot uses 3" radius wheels.
                0.7112, // The track width is 0.7112 meters.
                // The standard deviations for measurement noise:
                // x and y: 0.001 m
                // heading: 0.001 rad
                // l and r velocity: 0.1 m/s
                // l and r position: 0.005 m
                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
*/
    m_gyro.reset();
    
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | org.json.simple.parser.ParseException e) {
      e.printStackTrace();
      throw new RuntimeException(e);
    }

    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);
    poseEstimator =
            new SwerveDrivePoseEstimator(
                    m_kinematics,
                    m_gyro.getRotation2d(),
                    getModulePositions(),
                    new Pose2d(),
                    stateStdDevs,
                    visionStdDevs);

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

  }
  
    public void ResetDrives () {
  
      /* 
      m_frontLeft.resetEncoder();
      m_frontRight.resetEncoder();
      m_backLeft.resetEncoder();
      m_backRight.resetEncoder();
      */
      m_gyro.reset();
  
      SmartDashboard.putString("Gyro has been reset", java.time.LocalTime.now().toString());
    }
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    
    public void driveRobotRelative(ChassisSpeeds chassisSpeedsIn) {
      drive(chassisSpeedsIn.vxMetersPerSecond, chassisSpeedsIn.vyMetersPerSecond, chassisSpeedsIn.omegaRadiansPerSecond, false);
    }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    //if(RobotState.isTest()) {
      SmartDashboard.putString("gyro", m_gyro.getRotation2d().toString());

      SmartDashboard.putString("module 0", swerveModuleStates[0].toString());
      SmartDashboard.putString("module 1", swerveModuleStates[1].toString());
      SmartDashboard.putString("module 2", swerveModuleStates[2].toString());
      SmartDashboard.putString("module 3", swerveModuleStates[3].toString());

   // }
    xSpeed_cur = xSpeed;
    ySpeed_cur = ySpeed;
    rot_cur = rot;
  }

  public void stop() {
    drive(0, 0, 0, true);
  }

  public ChassisSpeeds getSpeed() {
    return new ChassisSpeeds( xSpeed_cur, ySpeed_cur, rot_cur);
  }

  /** Updates the field relative position of the robot. */
  // public void updateOdometry() {
  //   m_odometry.update(
  //       m_gyro.getRotation2d(),
  //       getModulePositions());
  // }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  } 

  public void resetPose(Pose2d aPose2d) {
//    m_odometry.resetPosition(m_gyro.getRotation2d(),
//         getModulePositions(), aPose2d );
    poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), aPose2d);
  }
  
  /* adding some gyro output so other classes don't need to access it directly */
  public double getYaw() {
    return (m_gyro.getAngle());
  }

  public Rotation2d getFacing() {
    return m_gyro.getRotation2d();
  }

  

    public PathConstraints getChassisConstrains() {
        return new PathConstraints(
          3.000,
          3.000,
          Units.degreesToRadians(540.000),
          Units.degreesToRadians(720.000)
        );
    }

  @Override
  public void periodic() {
      poseEstimator.update(m_gyro.getRotation2d(), getModulePositions());
      field.setRobotPose(poseEstimator.getEstimatedPosition());
    
      SmartDashboard.putData("robotpose", field);
      
      /*super.simulationPeriodic();

      // this is probably not the right speed calculation.. need to figure out where to get the speed
      m_frontLeft.getDriveMotorSim().iterate(xSpeed_cur + ySpeed_cur, m_frontLeft.getDriveMotorSim().getBusVoltage(), 0.02);
      m_frontRight.getDriveMotorSim().iterate(xSpeed_cur + ySpeed_cur, m_frontRight.getDriveMotorSim().getBusVoltage(), 0.02);

      simDrive.setInputs(m_frontLeft.getDriveMotorSim().getAppliedOutput() * m_frontLeft.getDriveMotorSim().getBusVoltage(),  m_frontRight.getDriveMotorSim().getAppliedOutput() * m_frontRight.getDriveMotorSim().getBusVoltage());
      simDrive.update(0.02);
      
      metricsProvider.updateLocation(simDrive.getPose());*/
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }
}
