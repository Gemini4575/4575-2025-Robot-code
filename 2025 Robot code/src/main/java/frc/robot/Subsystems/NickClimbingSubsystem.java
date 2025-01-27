package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbingConstants;

public class NickClimbingSubsystem extends SubsystemBase{
    
    SparkMax ClimbingMotor1;
    SparkMax ClimbingMotor2;

    public NickClimbingSubsystem() {
        // Initialization code here
        ClimbingMotor1 = new SparkMax(ClimbingConstants.ClimbingMotor1, MotorType.kBrushless);
        ClimbingMotor2 = new SparkMax(ClimbingConstants.ClimbingMotor2, MotorType.kBrushless);
    }

    private boolean Climb1() {
        // Code to move the elevator
        if(ClimbingMotor1.getEncoder().getPosition() < ClimbingConstants.ClimbingMotorPoseition) {
        ClimbingMotor1.set(ClimbingConstants.ClimbingSpeed);
        } else {
        return true;
        }
        return false;
    }

    private boolean Climb2() {
        // Code to move the elevator
        if(ClimbingMotor2.getEncoder().getPosition() < ClimbingConstants.ClimbingMotorPoseition) {
        ClimbingMotor2.set(ClimbingConstants.ClimbingSpeed);
        } else {
        return true;
        }
        return false;
    }

    public boolean Climb() {
        // Code to move the elevator
        if(Climb1() && Climb2()) {
            return true;
        }
        return false;
    }

    @Deprecated
    public void JoyClimb(double Joy) {
        ClimbingMotor1.set(Joy);
        ClimbingMotor2.set(Joy);
    }

    public void Stop() {
        // Code to stop the elevator
        ClimbingMotor1.set(0);
        ClimbingMotor2.set(0);
    }

}
