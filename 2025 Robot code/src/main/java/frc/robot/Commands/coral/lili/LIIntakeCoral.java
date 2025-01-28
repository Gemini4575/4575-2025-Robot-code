package frc.robot.commands.coral.lili;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LiliCoralSubystem;

public class LIIntakeCoral extends Command{
    LiliCoralSubystem c;
    public LIIntakeCoral(LiliCoralSubystem cc) {
        this.c = cc;
        addRequirements(c);
    }

    @Override
    public void execute() {
        if(c.intakeCoral()) {
            this.end(false);
        }
    }
}
