package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OzzyGrabberConstants;

public class OzzyGrabberSubsystem extends SubsystemBase{
    SparkMax GrabberMotor;
    SparkMax PoseMotor;
    DigitalInput AlgeaSensor;

    public OzzyGrabberSubsystem() {
        // Initialization code here
        GrabberMotor = new SparkMax(OzzyGrabberConstants.GrabberMotor, MotorType.kBrushless);
        PoseMotor = new SparkMax(OzzyGrabberConstants.PosetionMotor, MotorType.kBrushless);
        AlgeaSensor = new DigitalInput(OzzyGrabberConstants.BeamBreak);
    }

    public boolean Grab() {
        // Code to move the elevator
        if(PoseMotor.getEncoder().getPosition() < OzzyGrabberConstants.MiddleLength) {
            PoseMotor.set(OzzyGrabberConstants.DownSpeed);
        } else {
            PoseMotor.set(0);
        }
        return PoseMotor.getEncoder().getPosition() == OzzyGrabberConstants.MiddleLength;
    }

    public boolean intake() {
        if(AlgeaSensor.get()) {
            GrabberMotor.set(OzzyGrabberConstants.IntakeSpeed);
        } else {
            GrabberMotor.set(0);
        }
        return AlgeaSensor.get();
    }

    public boolean outake() {
        if(!AlgeaSensor.get()) {
            GrabberMotor.set(OzzyGrabberConstants.OutakeSpeed);
        } else {
            GrabberMotor.set(0);
        }
        return !AlgeaSensor.get();
    }

    public boolean down() {
        if(PoseMotor.getEncoder().getPosition() < OzzyGrabberConstants.MovmentLength) {
            PoseMotor.set(OzzyGrabberConstants.DownSpeed);
        } else {
            PoseMotor.set(0);
        }
        return PoseMotor.getEncoder().getPosition() == OzzyGrabberConstants.MovmentLength;
    }

    public boolean up() {
        if(PoseMotor.getEncoder().getPosition() > 0) {
            PoseMotor.set(OzzyGrabberConstants.UpSpeed);
        } else {
            PoseMotor.set(0);
        }
        return PoseMotor.getEncoder().getPosition() == 0;
    }

    public BooleanSupplier BeamBreak() {
        return () -> AlgeaSensor.get();
    }

    public BooleanSupplier FalseBeamnBreak() {
        return () -> !AlgeaSensor.get();
    }
}
