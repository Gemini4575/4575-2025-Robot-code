package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.NickClimbingSubsystem;

public class TEST extends Command{
    NickClimbingSubsystem c;
    
    public TEST(NickClimbingSubsystem c) {
        this.c = c;
        addRequirements(c);
    }
    boolean isFinished;
    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void execute() {
        if(c.test()) {
            c.Stop();
            isFinished = true;
        }
    }

    
}
