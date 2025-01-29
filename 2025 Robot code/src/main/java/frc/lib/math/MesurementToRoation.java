package frc.lib.math;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class MesurementToRoation {

    private double calculateRotations(double inches, double gearboxRatio) {
        double shaftCircumference = Math.PI * 0.5; // Circumference of the 1/2 inch shaft
        double rotationsForInches = inches / shaftCircumference; // Rotations needed for given inches
        double rotationsWithGearbox = rotationsForInches * gearboxRatio; // Adjust for gearbox
        return rotationsWithGearbox;
    }

    /**
     * Calculate the ticks needed to move a certain distance
     * 
     * @param inches       amount of inches to move
     * @param gearboxRatio the ratio of the gearbox on the motor
     * @return encoder ticks needed to move the distance
     */
    private double calculateTicks(double inches, double gearboxRatio) {
        double rotations = calculateRotations(inches, gearboxRatio);
    //  double ticksPerRotation = 42.0; // Ticks per one rotation per rev website not Mr.Fran
        return rotations;
    }

    public double calculateRotationsIN(double inches, double gearboxRatio) {
        return calculateTicks(inches, gearboxRatio);
    }

    public double calculateRotationsM(double meters, double gearboxRatio) {
        return calculateTicks(Units.metersToInches(meters), gearboxRatio);
    }

    public double calculateRotationsCM(double centemeters, double gearboxRatio) {
        return calculateTicks(Units.metersToInches(centemeters/100), gearboxRatio);
    }

    public double calculateRotationsFT(double feet, double gearboxRatio) {
        return calculateTicks(feet * 12, gearboxRatio);
    }

}
