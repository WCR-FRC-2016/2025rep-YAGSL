package frc.robot.commands.swervedrive.auto;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.ClawSubsystem.ClawSubsystem;

public class OutTakeAuto extends Command {
    ClawSubsystem clawSubsystem;

    Timer timer;


    public OutTakeAuto(ClawSubsystem clawSubsystem){
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize(){
        timer.start();
    }

    @Override
    public void execute(){
        if(timer.get() < 5)
        clawSubsystem.kickerMove(0.95);
    }

    @Override
    public boolean isFinished(){
        if(timer.get() >= 5){
            clawSubsystem.stopKicker();
            return true;
        }
        return false;
    }
}
