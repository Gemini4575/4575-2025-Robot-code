package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiliCoralConstants;

public class LiliCoralSubystem extends SubsystemBase{
    Spark gate;
    DigitalInput top;
    DigitalInput bottom;
    DigitalInput coral;

    public LiliCoralSubystem(){
        gate = new Spark(LiliCoralConstants.CoarlMotor);
        top = new DigitalInput(LiliCoralConstants.Top);
        bottom = new DigitalInput(LiliCoralConstants.Bottom);
        coral = new DigitalInput(LiliCoralConstants.Coral);
    }

    private boolean top() {
        return top.get();
    }

    private boolean bottom() {
        return bottom.get();
    }

    private boolean coral() {
        return coral.get();
    }

    public boolean intakeCoral() {
        if(!top()) {
            gate.set(LiliCoralConstants.GateSpeed);
        } else {
            gate.set(0);
        }
        return coral();
    }

    public boolean placeCoral() {
        if(!coral()) {
            return true;
        }
        if(!bottom()) {
            gate.set(LiliCoralConstants.GateSpeed * -1);
        } else {
            gate.set(0);
        }
        return coral();
    }

    public boolean DropGate() {
        if(!bottom()) {
            gate.set(LiliCoralConstants.GateSpeed * -1);
        } else {
            gate.set(0);
        }
        return bottom();
    }

    public BooleanSupplier Coral() {
        return () -> coral();
    }

}
