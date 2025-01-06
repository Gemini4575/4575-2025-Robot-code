package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constanst;

public class ArmSubsystem extends SubsystemBase {

    SparkMax armMotor;

    private final ProfiledPIDController ArmPidController =
      new ProfiledPIDController(
          Constanst.ArmConstants.P,
          Constanst.ArmConstants.I,
          Constanst.ArmConstants.D,
          new TrapezoidProfile.Constraints(
              Constanst.ArmConstants.maxSpeed, Constanst.ArmConstants.maxAcceleration));

    public ArmSubsystem() {
        armMotor = new SparkMax(Constanst.ArmConstants.ARM_MOTOR, MotorType.kBrushless);
    }
    
    public void moveArm(double speed) {
        armMotor.set(speed);
    }

    public void L1() {
        // Move the arm
    }
    
    public void L2() {
        // Stop the arm
    }
    
    public void L3() {
        // Reset the arm
    }

    public void L4(){
        // Move the arm to a specific position
    }
    
}
