package frc.robot.commands.coral.lili;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LiliCoralSubystem;

public class EXODropGate extends Command{
    LiliCoralSubystem c;
    public EXODropGate(LiliCoralSubystem cc){
        this.c = cc;
        addRequirements(c);
    }

    @Override
    public void execute() {
        if(c.DropGate()) {
            this.end(false);
        }
    }
}
