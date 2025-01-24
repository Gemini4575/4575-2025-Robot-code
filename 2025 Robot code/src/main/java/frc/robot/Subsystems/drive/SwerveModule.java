// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import java.util.function.Supplier;

import com.revrobotics.REVLibError;
// import com.revrobotics.spark.SparkSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class SwerveModule extends Command {

  //TODO verify these values
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;  
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec
  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double odometryFrequency = 100.0; // Hz
  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  private boolean sparkStickyFault = false;

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
  private final Rotation2d zeroRotation;

  // private final SparkSim driveMotorSim;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final RelativeEncoder m_driveEncoder;
  private final AnalogInput m_turningEncoder;

  private double encoderOffset = 0;
  
  private int moduleNumber = 0;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor and turning
   * encoder.
   *
   * @param driveMotorChannel      PWM output for the drive motor.
   * @param turningMotorChannel    PWM output for the turning motor.
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   */
  public SwerveModule(SwerveModuleConstants moduleConstants) {
    m_driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    // driveMotorSim = new SparkSim(m_driveMotor, DCMotor.getNEO(1));
    m_turningMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = new AnalogInput(moduleConstants.cancoderID);

    moduleNumber = moduleConstants.cancoderID;

    // load the encoder offset
    // encoderOffset = Preferences.getDouble("encoder" + moduleNumber,
    // encoderOffset);
    // hard coding the offset because its better?
    switch (moduleNumber) {
      case 0:
        encoderOffset = -4.168;// 3.769512307;//3.201315307;
        break;
      case 1:
        encoderOffset = -4.080; // 0.6210603266;// 6.9042456338;//5.333449307 + (Math.PI/2);
        break;
      case 2:
        encoderOffset = -2.910; // 4.9206722876;//0.2082833072 - (Math.PI/2);
        break;
      case 3:
        encoderOffset = -3.495; // 3.201315307;//3.769512307;
        break;
    }
    zeroRotation = new Rotation2d(encoderOffset);

    configureDrive();
    configureTurn();

    driveController = m_driveMotor.getClosedLoopController();
    turnController = m_turningMotor.getClosedLoopController();
  }

  private void configureDrive() {
    // Configure drive motor
    var driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            driveKp, 0.0,
            driveKd, 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        m_driveMotor,
        5,
        () ->
        m_driveMotor.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(m_driveMotor, 5, () -> m_driveEncoder.setPosition(0.0));
  }

  private void configureTurn() {
    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pidf(turnKp, 0.0, turnKd, 0.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        m_turningMotor,
        5,
        () ->
        m_turningMotor.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  // public SparkSim getDriveMotorSim() {
  //   return driveMotorSim;
  // }

  private double encoderValue() {
    var retVal = m_turningEncoder.getVoltage() / RobotController.getVoltage5V(); // convert voltage to %
    retVal = 2.0 * Math.PI * retVal; // get % of circle encoder is reading
    // SmartDashboard.putNumber("module " + moduleNumber, retVal);
    if (RobotState.isTest()) {
      SmartDashboard.putNumber("encoder raw " + moduleNumber, retVal);

      SmartDashboard.putNumber("encoder " + moduleNumber, (retVal * 1000) / 1000.0);
      SmartDashboard.putNumber("encoder degrees " + moduleNumber, (retVal * (180 / Math.PI) * 1000) / 1000.0);
    }
    retVal = (retVal + encoderOffset) % (2.0 * Math.PI); // apply offset for this encoder and map it back onto [0, 2pi]
    // might need this so we're in the same range as the pid controller is
    // expecting.
    // retVal = retVal - Math.PI;
    if (RobotState.isTest()) {
      SmartDashboard.putNumber("encoder adjusted " + moduleNumber, retVal);
    }
    return (retVal);
  }

  public void resetEncoder(){
    m_driveMotor.getEncoder().setPosition(0.0);

        // get the turning encoder and write it to preferences
    encoderOffset = m_turningEncoder.getVoltage() / RobotController.getVoltage5V(); // convert voltage to %
    encoderOffset = 2.0 * Math.PI * encoderOffset;    // get % of circle encoder is reading
    encoderOffset = (2.0 * Math.PI) - encoderOffset;
//    Preferences.initDouble("encoder" + moduleNumber, encoderOffset);
    Preferences.setDouble("encoder" + moduleNumber, encoderOffset);
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
   
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(encoderValue()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(encoderValue()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    var currentAngle = new Rotation2d(encoderValue());
    state.optimize(currentAngle);
    state.cosineScale(currentAngle);

    // Apply setpoints
    setDriveVelocity(state.speedMetersPerSecond / Constants.ChassisDefaultConfigs.DEFAULT_WHEEL_RADIUS_METERS);
    setTurnPosition(state.angle);
  }

  private void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  private void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), 0, 2.0 * Math.PI);
    turnController.setReference(setpoint, ControlType.kPosition);
  }
  
  /** Attempts to run the command until no error is produced. */
  private void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
      }
    }
  }

}
