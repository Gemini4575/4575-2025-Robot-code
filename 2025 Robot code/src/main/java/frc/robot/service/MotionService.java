package frc.robot.service;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.DriveStraight;
import frc.robot.commands.drive.Turn;
import frc.robot.datamodel.MotionDirective;
import frc.robot.subsystems.drive.DriveTrain;

public class MotionService {

    private final DriveTrain driveTrain;

    private MotionDirective[] motions;
    private int currentStep = -1;
    private Command currentCommand;

    public MotionService(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    public synchronized void executeSequence(MotionDirective... motions) {
        this.motions = motions;
        this.currentStep = 0;
        SmartDashboard.putString("Motion Status", "Starting");
        startMotion();
    }

    public synchronized void stop(boolean force) {
        SmartDashboard.putString("Motion Status", "Stopping");
        if (force) {
            currentCommand.end(true);
        }
        motions = null;
        currentStep = -1;
    }

    public synchronized void periodic() {
        if (motions != null && currentStep >=0 && currentCommand != null && currentCommand.isFinished()) {
            if (currentStep < motions.length-1) {
                currentStep++;
                startMotion();
            } else {
                stop(false);
            }
        }
    }

    private void startMotion() {
        //TODO pass the actual distance to these commands
        switch (motions[currentStep].getType()) {
            case DRIVE:
                currentCommand = new DriveStraight(driveTrain);
                break;
            case TURN:
                currentCommand = new Turn(driveTrain);
                break;
        }
        publshStartStatus();
        CommandScheduler.getInstance().schedule(currentCommand);
    }

    private void publshStartStatus() {
        SmartDashboard.putString("Motion Index", String.valueOf(currentStep));
        SmartDashboard.putString("Starting motion", motions[currentStep].toString());
    }

}
