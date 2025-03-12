package frc.robot.commands.swervedrive.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.FunnelSubsystem.FunnelSubsystem;

public class MoveFunnelDown extends Command {
    FunnelSubsystem funnelSubsystem;
    double speed;

    public MoveFunnelDown(FunnelSubsystem funnelSubsystem, double speed){
        this.funnelSubsystem = funnelSubsystem;
        this.speed = speed;
        addRequirements(funnelSubsystem);
    }

    @Override
    public void execute(){

            funnelSubsystem.moveDown(speed);
        

    }

    @Override
    public void end(boolean interrupted){
        funnelSubsystem.stop();
        System.out.println("stoped moving funnel down");
    }
}
