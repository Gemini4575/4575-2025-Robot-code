package frc.robot.commands.coral.nora;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.NoraArmSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class IntakeCoral extends Command{
    
    private NoraArmSubsystem elevator;
    private VisionSubsystem vision;
    

    public IntakeCoral(NoraArmSubsystem subsystem, VisionSubsystem vision) {
        elevator = subsystem;
        this.vision = vision;
        addRequirements(elevator, vision);
    }

    @Override
    public void initialize() {
        // Initialization code here
    }

    @Override
    public void execute() {
        // Code to move the elevator
        if (elevator.intakeCoral(vision.InRange())) {
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Code to stop the elevator
        elevator.stop();
    }
}
