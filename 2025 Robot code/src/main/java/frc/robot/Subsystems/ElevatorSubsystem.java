package frc.robot.Subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constanst;

public class ElevatorSubsystem extends SubsystemBase {

    SparkMax armMotor;

    private final ProfiledPIDController ArmPidController =
      new ProfiledPIDController(
          Constanst.ArmConstants.P,
          Constanst.ArmConstants.I,
          Constanst.ArmConstants.D,
          new TrapezoidProfile.Constraints(
              Constanst.ArmConstants.maxSpeed, Constanst.ArmConstants.maxAcceleration));

    public ElevatorSubsystem() {
        armMotor = new SparkMax(Constanst.ArmConstants.ARM_MOTOR, MotorType.kBrushless);
    }

    public void stop() {
        armMotor.set(0);
    }

    public void L1() {
        // Move the arm
        moveArm(armMotor.getAbsoluteEncoder(), Constanst.ArmConstants.L1Position);
    }
    
    public void L2() {
        // Stop the arm
        moveArm(armMotor.getAbsoluteEncoder(), Constanst.ArmConstants.L2Position);
    }
    
    public void L3() {
        // Reset the arm
        moveArm(armMotor.getAbsoluteEncoder(), Constanst.ArmConstants.L3Position);
    }

    public void L4(){
        // Move the arm to a specific position
        moveArm(armMotor.getAbsoluteEncoder(), Constanst.ArmConstants.L4Position);
    }

    public void moveArm(SparkAbsoluteEncoder encoder, double position) {
        final 
        double ArmOutput = ArmPidController.calculate(encoder.getPosition(), position);
        armMotor.set(ArmOutput);
    }
    
}
