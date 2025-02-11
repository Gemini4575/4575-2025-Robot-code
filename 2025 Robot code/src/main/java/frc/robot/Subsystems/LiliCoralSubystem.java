package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiliCoralConstants;

public class LiliCoralSubystem extends SubsystemBase{
    SparkMax gate;
    DigitalInput top;
    DigitalInput bottom;
    DigitalInput coral;
    Timer timer = new Timer();
    // 0.540528
    public LiliCoralSubystem(){
        gate = new SparkMax(LiliCoralConstants.CoarlMotor, MotorType.kBrushed);
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
    /**
     * SHOULD NOT BE USED IN COMP
     * @param joy the joystick axis your using
     */
    @Deprecated
    public void JoyControll(double joy) {
        gate.set(joy);
    }

    public BooleanSupplier Coral() {
        return () -> coral();
    }

}
