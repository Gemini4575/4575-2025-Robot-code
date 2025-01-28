package frc.robot.commands.coral.nora;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.NoraArmSubsystem;

public class CoralStation extends Command {

    private final NoraArmSubsystem elevator;

    public CoralStation(NoraArmSubsystem subsystem) {
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
        if (elevator.CoralStation()) {
            end(false);
        }
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
