package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class L2 extends Command {

    private final ElevatorSubsystem elevator;

    public L2(ElevatorSubsystem subsystem) {
        elevator = subsystem;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // Initialization code here
    }

    @Override
    public void execute() {
        // Code to move the elevator
        elevator.L2();
    }

    @Override
    public void end(boolean interrupted) {
        // Code to stop the elevator
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        // Condition to end the command
        return false;
    }
    }