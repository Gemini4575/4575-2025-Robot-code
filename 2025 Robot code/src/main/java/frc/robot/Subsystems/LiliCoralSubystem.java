package frc.robot.subsystems;

import static frc.robot.datamodel.MotionDirective.turn;

import java.util.function.BooleanSupplier;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiliCoralConstants;

public class LiliCoralSubystem extends SubsystemBase{

    private static enum State {OPEN, CLOSED, CLOSING, OPENING}

    SparkMax gate;
    //DigitalInput top;
    DigitalInput bottom;
    DigitalInput coral;
    Timer timer = new Timer();

    private State lastKnownState = State.CLOSED;

    // 0.540528
    public LiliCoralSubystem(){
        gate = new SparkMax(LiliCoralConstants.CoarlMotor, MotorType.kBrushed);
    //    top = new DigitalInput(LiliCoralConstants.Top);
        bottom = new DigitalInput(LiliCoralConstants.Top);
        coral = new DigitalInput(LiliCoralConstants.Coral);
        lastKnownState = State.CLOSED;
        first = true;
    }

    // private boolean top() {
    //     return top.get();
    // }

    private boolean bottom() {
        return bottom.get();
    }

    private boolean coral() {
        return coral.get();
    }

    // public boolean intakeCoral() {
    //     if ((lastKnownState == State.CLOSED || lastKnownState == State.CLOSING) && bottom()) {
    //         gate.set(0);
    //         lastKnownState = State.CLOSED;
    //         return coral();
    //     }
    //     gate.set(LiliCoralConstants.GateSpeed);
    //     lastKnownState = State.CLOSING;
    //     return false;
    // }

    // public boolean placeCoral() {
    //     if(!coral()) {
    //         gate.set(0);
    //         return true;
    //     }
    //     if ((lastKnownState == State.OPEN || lastKnownState == State.OPENING) && bottom()) {
    //         gate.set(0);
    //         lastKnownState = State.OPEN;
    //     } else {
    //         gate.set(-LiliCoralConstants.GateSpeed);
    //         lastKnownState = State.OPENING;
    //     }
    //     return false;
    // }

    // public boolean DropGate() {
    //     if(bottom() && lastKnownState == State.CLOSED) {
    //         gate.set(LiliCoralConstants.GateSpeed *-1);
    //     } else {
    //         lastKnownState = State.OPEN;
    //         gate.set(0);
    //     }
    //     return coral();
    // }

    // public boolean CloseGate() {
    //     if(bottom() && lastKnownState == State.OPEN) {
    //         gate.set(LiliCoralConstants.GateSpeed);
    //     } else {
    //         lastKnownState = State.CLOSED;
    //         gate.set(0);
    //         return true;
    //     }
    //     return false;
    // }
    boolean first = true;
    private void start() {
        if(first) {
            timer.reset();
            timer.start();
            first = false;
        }
    }

    public boolean DropGate() {
        start();
        if(timer.advanceIfElapsed(0.5) && coral()) {
            gate.stopMotor();
            first = true;
        } else {
            gate.set(LiliCoralConstants.GateSpeed);
        }

        return !coral();
    }

    public boolean CloseGate() {
        start();
        if(timer.advanceIfElapsed(0.5)) {
            gate.stopMotor();
            first = true;
            return true;
        } else {
            gate.set(-LiliCoralConstants.GateSpeed);
            return false;
        }
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
    

    @Override
    public void periodic() {
        if(first) {
            if(bottom() && lastKnownState == State.CLOSED) {
                
            }
        }
    }

}
