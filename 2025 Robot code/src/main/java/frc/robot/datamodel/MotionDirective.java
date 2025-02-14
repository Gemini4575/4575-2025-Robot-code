package frc.robot.datamodel;

public class MotionDirective {
    public static enum MotionType {
        DRIVE,
        TURN,
        DROP_CORAL
    }
    
    private final MotionType type;

    private final double amount;

    private MotionDirective(MotionType type, double amount) {
        this.type = type;
        this.amount = amount; 
    }

    public static MotionDirective turn(double amount) {
        return new MotionDirective(MotionType.TURN, amount);
    }
    public static MotionDirective drive(double amount) {
        return new MotionDirective(MotionType.DRIVE, amount);
    }
    public static MotionDirective dropCoral() {
        return new MotionDirective(MotionType.DROP_CORAL, 0);
    }
    
    public MotionType getType() {
        return type;
    }

    public double getAmount() {
        return amount;
    }

    @Override
    public String toString() {
        return "[type=" + type.name() + ", amount=" + amount + "]";
    }

    
}
