package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Motor;
import frc.robot.Constants.OzzyGrabberConstants;

public class OzzyGrabberSubsystem extends SubsystemBase{
    Motor GrabberMotor;
    Motor PoseMotor;
    DigitalInput AlgeaSensor;

    public OzzyGrabberSubsystem() {
        // Initialization code here
        GrabberMotor = new Motor(OzzyGrabberConstants.GrabberMotor, MotorType.kBrushless);
        PoseMotor = new Motor(OzzyGrabberConstants.PosetionMotor, MotorType.kBrushed);
        AlgeaSensor = new DigitalInput(OzzyGrabberConstants.BeamBreak);

    }

    public boolean Grab() {
        // Code to move the elevator
        if(PoseMotor.getPosition() < OzzyGrabberConstants.MiddleLength) {
            PoseMotor.set(OzzyGrabberConstants.DownSpeed);
        } else {
            PoseMotor.set(0);
        }
        return PoseMotor.getPosition() == OzzyGrabberConstants.MiddleLength;
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
        if(PoseMotor.getPosition() < OzzyGrabberConstants.MovmentLength) {
            PoseMotor.set(OzzyGrabberConstants.DownSpeed);
        } else {
            PoseMotor.set(0);
        }
        return PoseMotor.getPosition() == OzzyGrabberConstants.MovmentLength;
    }

    public boolean up() {
        if(PoseMotor.getPosition() > 0) {
            PoseMotor.set(OzzyGrabberConstants.UpSpeed);
        } else {
            PoseMotor.set(0);
        }
        return PoseMotor.getPosition() == 0;
    }

    public BooleanSupplier BeamBreak() {
        return () -> AlgeaSensor.get();
    }

    public BooleanSupplier FalseBeamnBreak() {
        return () -> !AlgeaSensor.get();
    }
}
