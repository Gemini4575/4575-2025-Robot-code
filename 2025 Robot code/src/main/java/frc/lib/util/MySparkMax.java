package frc.lib.util;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class MySparkMax{
    SparkMax s;
    SparkMaxConfig sc;
    public MySparkMax(int CANID, MotorType a) {
        s = new SparkMax(CANID, a);
        sc = new SparkMaxConfig();
    }

    public double getPosition() {
        return s.getEncoder().getPosition();
    }

    public void set(double value) {
        s.set(value);
    }

    public void stop() {
        s.set(0);
    }
    
    public void setInverted() {
        sc.inverted(true);
        s.configure(sc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void resetEncoder(){
        s.getEncoder().setPosition(0);
    }

    public void setVoltage(double value) {
        s.setVoltage(value);
    }

    public RelativeEncoder getEncoder() {
        return s.getEncoder();
    }

    public void configure(SparkBaseConfig c, ResetMode e, PersistMode d) {
        s.configure(c, e, d);
    }
}
